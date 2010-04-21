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


#include "TI_AdapterApiC.h"
#include "osDot11.h"

#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>


#include "g_tester.h"
#include "console.h"
#include "cu_cmd.h"
#include "ticon.h"
#include "ipc.h"

#include "paramOut.h"

/************/
/* Defines */
/**********/					    


#define G_TESTER_STATUS_OK		(0)
#define G_TESTER_STATUS_ERROR	(1)

#define G_TESTER_GWSI_INIT_NO_ERROR						(0x00)
#define G_TESTER_GWSI_INIT_ERROR_READING_FW_FILE_LENGTH	(0x01)
#define G_TESTER_GWSI_INIT_ERROR_OPENING_FW_FILE		(0x02)
#define G_TESTER_GWSI_INIT_ERROR_READING_EP_FILE_LENGTH	(0x03)
#define G_TESTER_GWSI_INIT_ERROR_OPENING_EP_FILE		(0x04)
#define G_TESTER_GWSI_INIT_ERROR_READING_EP_FILE		(0x05)
#define G_TESTER_GWSI_INIT_ERROR_READING_FW_FILE		(0x06)
#define G_TESTER_GWSI_INIT_ERROR_ALLOCATION_MEMORY		(0x07)

#define G_TESTER_GWSI_GET_INIT_TABLE_NO_ERROR				(0x00)
#define G_TESTER_GWSI_GET_INIT_TABLE_ERROR_NO_FILE_NAME		(0x01)
#define G_TESTER_GWSI_GET_INIT_TABLE_ERROR_WRITING_TO_FILE	(0x02)
#define G_TESTER_GWSI_GET_INIT_TABLE_ERROR_CREATING_FILE	(0x03)
#define G_TESTER_GWSI_GET_INIT_TABLE_ERROR_TABLE_NOT_AVAIL	(0x04)

#define G_TESTER_GWSI_CONFIG_NO_ERROR					(0x00)
#define G_TESTER_GWSI_CONFIG_ERROR_READING_FILE			(0x01)
#define G_TESTER_GWSI_CONFIG_ERROR_ALLOCATING_MEMORY	(0x02)
#define G_TESTER_GWSI_CONFIG_ERROR_GETIING_FILE_SIZE	(0x03)
#define G_TESTER_GWSI_CONFIG_ERROR_OPENING_FILE			(0x04)



extern void quit_func(void);	
void g_tester_process_general_cmd_run_cmd(unsigned char *cmd_buffer);
void g_tester_process_get_init_table(unsigned char *cmd_buffer);

void g_tester_send_to_host(tiUINT8 *buffer, tiUINT32 length);
void g_tester_cmd_status(void);
void g_tester_bssid_list(void);
void g_tester_send_received_event(unsigned int cmd_op, tiUINT8 status);
void g_tester_process_gwsi_init_cmd(unsigned char *cmd_buffer);
void g_tester_process_gwsi_config_cmd(unsigned char *cmd_buffer);
void g_tester_process_gwsi_release_cmd(unsigned char *cmd_buffer);
void g_tester_process_gwsi_cmd(unsigned char *cmd_buffer);
void g_tester_register_event(tiUINT16 event_mask);
void g_tester_unregister_event(tiUINT16 event_mask);
void g_tester_cmd_debug_driver_print(unsigned char *cmd_buffer);
void g_tester_cmd_get_version(void);
void g_tester_set_rate(tiUINT8 rate_index);
void g_tester_wep_add_key(unsigned char *cmd_buffer);
int g_tester_gwsi_event_handler(IPC_EV_DATA* pData);
void g_tester_cmd_plt_register_read(UINT32 uiRegAddress);
void g_tester_cmd_plt_RxPer_GetResults(void);
void g_tester_cmd_plt_mib_read(unsigned char *cmd_buffer);
void g_tester_cmd_plt_mib_write(unsigned char *cmd_buffer);
void g_tester_cmd_get_defaultWEPKey(void);
void g_tester_Roaming_candidates_list(void);
void g_tester_scAn__configApp__Display(void);
void g_tester_plt_calibration_get_nvs_buffer(void);


TI_HANDLE gwsi_event_id;


/************************************************************************
*                        g_tester_send_to_host                         *
************************************************************************
DESCRIPTION:  

  CONTEXT    : 
************************************************************************/
void g_tester_send_to_host(tiUINT8 *buffer, tiUINT32 length)
{
    /* console_printf_terminal("g_tester, g_tester_send_to_host (length = %d)!\n", length); */
    
    /* Send the buffer to the host */
    console_send_buffer_to_host(ETHERNET_UTILS_G_TESTER_MODULE_ID, buffer, length);
}

/************************************************************************
*                        g_tester_init			                        *
************************************************************************
DESCRIPTION:  

  CONTEXT    : 
************************************************************************/
void g_tester_init()
{
    /************************************/
    /* Register the GWSI event handler */
    /**********************************/
    
    IPC_EVENT_PARAMS pEvent;
    
    
    pEvent.uEventType       = IPC_EVENT_GWSI;
    pEvent.uDeliveryType    = DELIVERY_PUSH;
    pEvent.pfEventCallback  = g_tester_gwsi_event_handler;
    pEvent.hUserParam		= 0;
    
    /* Register the event, set the pEvent.uEventID and the pEvent.uProcessID */ 
    if(!TI_RegisterEvent(g_id_adapter, &pEvent))
    {
        gwsi_event_id = pEvent.uEventID;
    }
    else
    {
        console_printf_terminal("g_tester,  g_tester_init. ERROR Registering GWSI event\n");
    }
    
}

/************************************************************************
*                        g_tester_deinit		                        *
************************************************************************
DESCRIPTION:  

  CONTEXT    : 
************************************************************************/
void g_tester_deinit()
{
    /**************************************/
    /* Unregister the GWSI event handler */
    /************************************/
    
    IPC_EVENT_PARAMS pEvent;
    
    pEvent.uEventType = IPC_EVENT_GWSI;
    pEvent.uEventID   = gwsi_event_id;
    TI_UnRegisterEvent(g_id_adapter, &pEvent);
}

/************************************************************************
*                        wipp_control_check_command                    *
************************************************************************
DESCRIPTION: Handle the wipp control specific commands 

  CONTEXT    : main process only! 			 
************************************************************************/
unsigned char g_tester_check_command(unsigned char *input_string)
{
    unsigned char return_value = FALSE;
    unsigned int cmd_op; 
    /*ConParm_t parm;*/
    ConParm_t parms[12];
    int parms_num;
    unsigned char* data_string = input_string + 5;
    
    if (input_string[0] == '-')
    {
        /* Get the command opcode */
        cmd_op = (input_string[1] | (input_string[2] << 8));
        
        console_printf_terminal("g_tester,  g_tester_check_command (OpCode = 0x%x).\n", cmd_op);
        
        /* Notify the host that we got the event */
        g_tester_send_received_event(cmd_op, G_TESTER_STATUS_OK);
        
        if (G_TESTER_IS_GENERAL_GROUP_CMD(cmd_op))
        {
            /************************************/
            /* This command is GENERAL command */
            /**********************************/
            
            switch (cmd_op)
            {
            case G_TESTER_GENERAL_CMD_RUN_CMD:
                g_tester_process_general_cmd_run_cmd(data_string);
                break;
                
            case G_TESTER_GENERAL_CMD_GET_INIT_T:
                g_tester_process_get_init_table(data_string);
                break;
                
            default:
                console_printf_terminal("g_tester (general switch) - unsupported command!\n");
                break;
            }
        }		   
        else if (G_TESTER_IS_GWSI_GROUP_CMD(cmd_op))
        {
            /*********************************/
            /* This command is GWSI command */
            /*******************************/
            
            switch (cmd_op)
            {
            case G_TESTER_GWSI_CMD_INITIALIZE:
                g_tester_process_gwsi_init_cmd(data_string);
                break;
                
            case G_TESTER_GWSI_CMD_CONFIG:
                g_tester_process_gwsi_config_cmd(data_string);
                break;
                
            case G_TESTER_GWSI_CMD_RELEASE:
                g_tester_process_gwsi_release_cmd(data_string);
                break;
                
            default:
                g_tester_process_gwsi_cmd(&input_string[1]);
                break;
            }		  
        }
        else
        {
            /********************************/
            /* This command is CLI command */
            /******************************/
            switch(cmd_op)
            {
            case G_TESTER_CLI_CMD_DRIVER__START:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_start_driver(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_DRIVER__STOP:
                cmd_stop_driver(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_DRIVER__STATUS:
                g_tester_cmd_status();
                break;
                
            case G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST:
                g_tester_bssid_list();
                break;
                
            case G_TESTER_CLI_CMD_CONNECTION__CONNECT:
                parms[0].value = (U32)(data_string);
                data_string += 32; /*Seek to the end of the string */
                parms[1].value = (U32)(data_string);
                /*Find the number of none empty strings */
                for(parms_num = 2; (parms_num >= 1) && (strlen((char*)parms[parms_num-1].value) == 0); parms_num--)
                {
                }
                cmd_connect(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_CONNECTION__DISASSOCIATE:
                cmd_disassociate(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_CONNECTION__STATUS:
                g_tester_cmd_status();
                break;
                
            case G_TESTER_CLI_CMD_CONNECTION__FULL_BSSID_LIST:
                cmd_Full_bssid_list(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__SSID:
                parms[0].value = (U32)(data_string);
                parms_num =(strlen((char*)parms[0].value) == 0)?0:1;
                cmd_modify_ssid(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__CHANNEL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_channel(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__RATE:
                g_tester_set_rate(input_string[5]);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__MODE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_bss_type(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__FRAG:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_frag_threshold(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__RTS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_rts_threshold(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__PREAMBLE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_preamble(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__SLOT:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_short_slot(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__INFO:
                cmd_get_selected_bssid_info(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__DRIVERSTATE:
                cmd_get_driver_state(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__SIGNAL:
                cmd_get_rsii_level(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__TX_POWER_LEVEL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_show_tx_power_level_table(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__TX_POWER_DBM:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_tx_power_dbm(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__D_ENABLEDISABLE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_enableDisable_802_11d(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__H_ENABLEDISABLE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_enableDisable_802_11h(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__D_COUNTRY_2_4IE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_d_Country_2_4Ie(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__D_COUNTRY_5IE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_d_Country_5Ie(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__ANTENNA__DIVERSITYPARAMS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                cmd_modify_antenna_diversity(parms, 5);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__BEACON__SET_BEACON_FILTER_MODE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Beacon_Filter_Set_Desired_State(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__ADVANCED__DRAFT:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_modify_ext_rates_ie(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_MANAGEMENT__ADVANCED__SUPPORTED_RATES:
                parms[0].value = (U32)(data_string);
                parms_num =(strlen((char*)parms[0].value) == 0)?0:1;
                cmd_modify_supported_rates(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_SHOW__STATISTICS:
                cmd_show_statistics(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SHOW__TX_STATISTICS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_show_tx_statistics(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_SHOW__ADVANCED:
                cmd_show_advanced_params(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__AUTHENTICATION:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_auth(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__EAP:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_eap(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__ENCRYPTION:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_encrypt(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__KEYTYPE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_key_type(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__MIXEDMODE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_mixed_mode(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__CREDENTIALS:
                parms[0].value = (U32)(data_string);
                data_string += 32; /*Seek to the end of the string */
                parms[1].value = (U32)(data_string);
                /*Find the number of none empty strings */				
                for(parms_num = 2; (parms_num >= 1) && (strlen((char*)parms[parms_num-1].value) == 0); parms_num--)
                {
                }
                cmd_privacy_credent(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__PSKPASSPHRASE:
                parms[0].value = (U32)(data_string);
                parms_num =(strlen((char*)parms[0].value) == 0)?0:1;
                cmd_privacy_PSKPassphrase(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__CERTIFICATE:
                parms[0].value = (U32)(data_string);
                data_string += 32; /*Seek top the end of the string */
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                cmd_privacy_certificate(parms, 2);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__WPA_OPTIONS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_wpa_options(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__WEP__ADD:
                g_tester_wep_add_key(data_string);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__WEP__REMOVE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_removekey(parms, 1);
                break;
                
#ifdef EXC_MODULE_INCLUDED
            case G_TESTER_CLI_CMD_PRIVACY__EXC__CONFIGURE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_exc_config(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__EXC__NETWORKEAP:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_privacy_exc_netEap(parms, 1);
                break;
#endif
            case G_TESTER_CLI_CMD_SCAN__START:
                cmd_Scan_Start(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__STOP:
                cmd_Scan_Stop(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGAPP__GLOBAL:
            {
                int numOfParam;
                parms[0].value = (U32)(data_string);                /*SSID*/
                data_string+=33; /*seek the end of the string */
                P_BUFFER_GET_UINT32(data_string, parms[1].value);   /*Scan Type*/
                P_BUFFER_GET_UINT32(data_string, parms[2].value);   /*Band*/
                P_BUFFER_GET_UINT32(data_string, parms[3].value);   /*Probe Request Number*/
                P_BUFFER_GET_UINT32(data_string, parms[4].value);   /*Probe Request Rate*/
#ifdef TI_DBG
                numOfParam = 7;
                P_BUFFER_GET_UINT32(data_string, parms[5].value);   /*Tid*/
                P_BUFFER_GET_UINT32(data_string, parms[6].value);   /*Number of Channels*/
#else
                numOfParam = 6;
                P_BUFFER_GET_UINT32(data_string, parms[5].value);   /* skip XML Tid*/
                P_BUFFER_GET_UINT32(data_string, parms[5].value);   /*Number of Channels*/
#endif

				 cmd_Scan_app_global_config(parms, numOfParam);
            }
                cmd_Scan_app_global_config(parms, 6);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGAPP__CHANNEL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                parms[1].value = (U32)(data_string);
                data_string += 18; /*seek the end of the string */
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                cmd_Scan_app_channel_config(parms, 8);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGAPP__CLEAR:
                cmd_Scan_app_clear(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGAPP__DISPLAY:
                cmd_Scan_app_display(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__GLOABAL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                cmd_Scan_policy_global_config(parms, 6);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__MISC:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                cmd_Scan_band_global_config(parms, 5);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__CHANNEL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                cmd_Scan_band_channel_config(parms, 3);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__TRACK:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                P_BUFFER_GET_UINT32(data_string, parms[8].value);
                P_BUFFER_GET_UINT32(data_string, parms[9].value);
                P_BUFFER_GET_UINT32(data_string, parms[10].value);
                cmd_Scan_band_track_config(parms, 11);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__DISCOVERY:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                P_BUFFER_GET_UINT32(data_string, parms[8].value);
                P_BUFFER_GET_UINT32(data_string, parms[9].value);
                P_BUFFER_GET_UINT32(data_string, parms[10].value);
                cmd_Scan_band_discover_config(parms, 11);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__IMMEDIATE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                P_BUFFER_GET_UINT32(data_string, parms[8].value);
                P_BUFFER_GET_UINT32(data_string, parms[9].value);
                P_BUFFER_GET_UINT32(data_string, parms[10].value);
                cmd_Scan_band_immed_config(parms, 11);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__DISPLAY:
                cmd_Scan_policy_display(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__CLEAR:
                cmd_Scan_policy_clear(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__STORE:
                cmd_Scan_policy_store(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BSSLIST:
                g_tester_Roaming_candidates_list();
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__ENABLE:
                if (input_string[5] == FALSE)
                {
                    cmd_Roaming_disable(NULL, 0);
                }
                else
                {
                    cmd_Roaming_enable(NULL, 0);
                }
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__LOW_PASS_FILTER:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_lowPassFilter(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__QUALITY_THRESHOLD:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_qualityIndicator(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__GET:
                cmd_Roaming_getConfParams(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__TX_RETRY:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_dataRetryThreshold(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__BSS_LOSS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_numExpectedTbttForBSSLoss(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__TX_RATE_THRESHOLD:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_txRateThreshold(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__LOW_RSSI_THRESHOLD:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_lowRssiThreshold(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__LOW_SNR_THRESHOLD:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_lowSnrThreshold(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__LOW_QUALITY_FOR_SCAN:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_lowQualityForBackgroungScanCondition(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__NORMAL_QUALITY_FOR_SCAN:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Roaming_normalQualityForBackgroungScanCondition(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__ADD:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                cmd_add_tspec(parms, 6);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__GET:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_get_tspec_params(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__DELETE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                cmd_delete_tspec(parms, 2);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__AP_PARAMS:
                cmd_get_ap_qos_params(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__AP_CAPABILITIES:
                cmd_get_ap_qos_capabilities(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__AC_STATUS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_get_ac_status(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__MEDIUM_USAGE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                cmd_medium_usage_threshold(parms, 3);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__PHY_RATE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                cmd_phy_rate_threshold(parms, 3);
                break;
                
            case G_TESTER_CLI_CMD_QOS__UPSD__DESIRED_PS_MODE:
                cmd_get_desired_ps_mode(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_QOS__CLASSIFIER__TXCLASSIFIER:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                P_BUFFER_GET_UINT32(data_string, parms[8].value);
                P_BUFFER_GET_UINT32(data_string, parms[9].value);
                P_BUFFER_GET_UINT32(data_string, parms[10].value);
                P_BUFFER_GET_UINT32(data_string, parms[11].value);
                cmd_config_tx_classifier(parms, 12);
                break;
                
            case G_TESTER_CLI_CMD_QOS__CLASSIFIER__INSERT:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                cmd_insert_clsfr_entry(parms, 7);
                break;
                
            case G_TESTER_CLI_CMD_QOS__CLASSIFIER__REMOVE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                cmd_remove_clsfr_entry(parms, 7);
                break;
                
            case G_TESTER_CLI_CMD_QOS__QOSPARAMS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                cmd_set_qos_params(parms, 4);
                break;
                
            case G_TESTER_CLI_CMD_QOS__POLL_AP_PACKETS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_poll_ap_packets(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_QOS__RX_TIMEOUT:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                cmd_set_rxTimeOut_params(parms, 2);
                break;
                
            case G_TESTER_CLI_CMD_POWER__SET_POWER_MODE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_set_power_mode(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_POWER__SET_POWERSAVE_POWERLEVEL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_set_PowerSave_PowerLevel(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_POWER__TRAFFIC_THRESHOLDS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                cmd_traffic_intensity_threshold(parms, 3);
                break;
                
            case G_TESTER_CLI_CMD_POWER__ENABLE:
                cmd_enable_traffic_events(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_POWER__DISABLE:
                cmd_disable_traffic_events(NULL, 0);
                break;
                          
            case G_TESTER_CLI_CMD_EVENTS__REGISTER:
                /* Register Event */
                g_tester_register_event(input_string[5] | (input_string[6] << 8));
                break;
                
            case G_TESTER_CLI_CMD_EVENTS__UNREGISTER:
                /* Unregister Event */
                g_tester_unregister_event(input_string[5] | (input_string[6] << 8));
                break;
                
#ifdef TI_DBG
                
            case G_TESTER_CLI_CMD_FILE__LOAD:
                parms[0].value = (U32)(data_string);
                parms_num =(strlen((char*)parms[0].value) == 0)?0:1;
                cmd_file_load(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_BT_COEXSISTANCE__ENABLE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_bt_coe_enable(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_BT_COEXSISTANCE__RATE:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                cmd_bt_coe_rate(parms, 8);
                break;
                
            case G_TESTER_CLI_CMD_BT_COEXSISTANCE__CONFIG:
				cmd_Scan_app_clear(NULL, 0);
				 g_tester_scAn__configApp__Display();
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                P_BUFFER_GET_UINT32(data_string, parms[5].value);
                P_BUFFER_GET_UINT32(data_string, parms[6].value);
                P_BUFFER_GET_UINT32(data_string, parms[7].value);
                P_BUFFER_GET_UINT32(data_string, parms[8].value);
                P_BUFFER_GET_UINT32(data_string, parms[9].value);
                P_BUFFER_GET_UINT32(data_string, parms[10].value);
                P_BUFFER_GET_UINT32(data_string, parms[11].value);
                P_BUFFER_GET_UINT32(data_string, parms[12].value);
                P_BUFFER_GET_UINT32(data_string, parms[13].value);
                cmd_bt_coe_config(parms, 14);
                break;
                
            case G_TESTER_CLI_CMD_BT_COEXSISTANCE__STATUS:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_bt_coe_get_status(parms, 1);
                break;
                
#ifdef EXC_MODULE_INCLUDED
            case G_TESTER_CLI_CMD_MEASUREMENT__ENABLE:
                cmd_Measurement_enable(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_MEASUREMENT__DISABLE:
                cmd_Measurement_disable(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_MEASUREMENT__MAX_DURATION:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_Measurement_setMaxDuration(parms, 1);
                break;
#endif
            case G_TESTER_CLI_CMD_REPORT__SET:
                parms[0].value = (U32)(data_string);
                parms_num =(strlen((char*)parms[0].value) == 0)?0:1;
                cmd_report_set(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_REPORT__ADD:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_report_add(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_REPORT__CLEAR:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_report_clear(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_REPORT__LEVEL:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_report_severity_level(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_DEBUG__REGISTER:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                cmd_hw_register(parms, 2);
                break;
                
            case G_TESTER_CLI_CMD_DEBUG__PRINT:
                g_tester_cmd_debug_driver_print(data_string);
                break;
                
            case G_TESTER_CLI_CMD_DEBUG__BUFFER:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                data_string += 9; /* seek to the end of the string */
                parms[1].value = (U32)(data_string);
                parms_num =(strlen((char*)parms[1].value) == 0)?1:2;
                cmd_debug_buffer_put(parms, parms_num);
                break;
                
            case G_TESTER_CLI_CMD_ROOT__ABOUT:
                /* Driver version */
                g_tester_cmd_get_version();  
                break;
                
            case G_TESTER_CLI_CMD_ROOT__QUIT:
                quit_func();
                break;
#endif 
                
            case G_TESTER_CLI_CMD_PLT__REGISTER__READ:
                {
                    UINT32 RegAddress;
                    P_BUFFER_GET_UINT32(data_string, RegAddress);
                    g_tester_cmd_plt_register_read(RegAddress);
                }
                break;
                
            case G_TESTER_CLI_CMD_PLT__REGISTER__WRITE:
                {
                    tiUINT32 uiRegAddress; 
                    tiUINT32 uiRegValue; 
                    P_BUFFER_GET_UINT32(data_string, uiRegAddress);
                    P_BUFFER_GET_UINT32(data_string, uiRegValue);
                    TI_PLT_WriteRegister(g_id_adapter, uiRegAddress, uiRegValue);
                }
                break;

            case G_TESTER_CLI_CMD_PLT_RADIO_TUNE:
                {
                    UINT32 status;
                    TestCmdChannelBand_t ChannelBand;
                    tiUINT8 return_buffer[3];
                    tiUINT8 *p_return_buffer = return_buffer;                    

                    P_BUFFER_GET_UINT32(data_string, ChannelBand.band);
                    P_BUFFER_GET_UINT32(data_string, ChannelBand.channel);
                    status = TI_PLT_RadioTune(g_id_adapter, &ChannelBand);
                    P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT_RADIO_TUNE, (UINT8)status);
                    g_tester_send_to_host(return_buffer, 3);
                }
                break;
                
            case G_TESTER_CLI_CMD_PLT__RX_PER__START:
                cmd_PLT_RxPerStart(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_PLT__RX_PER__STOP:
                cmd_PLT_RxPerStop(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_PLT__RX_PER__CLEAR:
                cmd_PLT_RxPerClear(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_PLT__RX_PER__GET_RESULTS:
                g_tester_cmd_plt_RxPer_GetResults();
                break;
                
            case G_TESTER_CLI_CMD_PLT__TX__CW:
                {
                    tiUINT8 return_buffer[3];
                    tiUINT8 *p_return_buffer = return_buffer;                    
                    tiUINT32 status;
                    TestCmdChannelBand_t PltTxCW;
                    P_BUFFER_GET_UINT32(data_string, PltTxCW.band);
                    P_BUFFER_GET_UINT32(data_string, PltTxCW.channel);
                    status = TI_PLT_TxCW(g_id_adapter, &PltTxCW);
                    P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT__TX__CW, (UINT8)status);
                    g_tester_send_to_host(return_buffer, 3);
                }
                break;
                
            case G_TESTER_CLI_CMD_PLT__TX__CONTINUES:
                {
                    
                    PltTxContinues_t PltTxContinues;
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.band);
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.chID);
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.rate);
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.preamble);
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.InterPacketDelay);
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.NumOfFrames);
                    P_BUFFER_GET_UINT8(data_string, PltTxContinues.mode);
                    
                    TI_PLT_TxContiues(g_id_adapter, &PltTxContinues);
                    
                    if (OK == TI_PLT_TxContiues(g_id_adapter, &PltTxContinues))
                        console_printf_terminal("PltTxContinues (band=%d, chID=%d, rate=%d, preamble=%d, InterPacketDelay=%d, NumOfFrames=%d, mode=0x%x)- OK\n",
                        PltTxContinues.band,             
                        PltTxContinues.chID,             
                        PltTxContinues.rate,             
                        PltTxContinues.preamble,         
                        PltTxContinues.InterPacketDelay, 
                        PltTxContinues.NumOfFrames,      
                        PltTxContinues.mode);
                    else
                        console_printf_terminal("PltTxContinues - NOK\n");                    
                }
                break;
                
            case G_TESTER_CLI_CMD_PLT__TX__STOP:
                cmd_PLT_TxStop(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_PLT__MIB__READ:
                g_tester_cmd_plt_mib_read(data_string);
                break;
                
            case G_TESTER_CLI_CMD_PLT__MIB__WRITE:
                g_tester_cmd_plt_mib_write(data_string);
                break;
                
            case G_TESTER_CLI_CMD_PRIVACY__WEP__GET:
                g_tester_cmd_get_defaultWEPKey();
                break;

            case G_TESTER_CLI_CMD_PLT__CALIBRATION__RX:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                P_BUFFER_GET_UINT32(data_string, parms[1].value);
                P_BUFFER_GET_UINT32(data_string, parms[2].value);
                P_BUFFER_GET_UINT32(data_string, parms[3].value);
                P_BUFFER_GET_UINT32(data_string, parms[4].value);
                cmd_PLT_RxCal(parms, 5);
                break;
                
            case G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__START:
				P_BUFFER_GET_UINT8(data_string, parms[0].value);
                cmd_PLT_TxCalStart(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__STOP:
                cmd_PLT_TxCalStop(NULL, 0);
                break;
                
            case G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__GAIN_GET:
                {
                    tiUINT8 return_buffer[10];
                    tiUINT8 *p_return_buffer = return_buffer;                    
                    
                    UINT32 status;
                    PltGainGet_t PLTGainGet;                     
                    status = TI_PLT_TxCalGainGet(g_id_adapter, &PLTGainGet);
                    P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__GAIN_GET, (UINT8)status);
                    if (status == OK)
                    {
                        P_BUFFER_ADD_UINT8(p_return_buffer, PLTGainGet.TxGain);
                        P_BUFFER_ADD_UINT8(p_return_buffer, PLTGainGet.TxUpperBound);
                        P_BUFFER_ADD_UINT8(p_return_buffer, PLTGainGet.TxLowerBound);
                    }
                    g_tester_send_to_host(return_buffer, p_return_buffer-return_buffer); 
                }
                break;
                
            case G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__GAIN_ADJUST:
                P_BUFFER_GET_UINT32(data_string, parms[0].value);
                cmd_PLT_TxCalGainAdjust(parms, 1);
                break;
                
            case G_TESTER_CLI_CMD_PLT__CALIBRATION__GET_NVS_BUFFER:
                g_tester_plt_calibration_get_nvs_buffer();
                break;
                
            default:
                console_printf_terminal("g_tester - unsupported command!\n");
                break;
            }
        }
        
        return_value = TRUE;
    }
    
    return return_value;
}

/************************************************************************
*                        g_tester_process_general_cmd_run_cmd          *
************************************************************************
DESCRIPTION: 

  CONTEXT    : main process only! 			 
  ************************************************************************/
  void g_tester_process_general_cmd_run_cmd(unsigned char *cmd_buffer)
  {
      tiUINT8 return_buffer[5];
      tiUINT8 *p_return_buffer = return_buffer;
      tiUINT16 return_value;
      
      console_printf_terminal("g_tester - Executing cmd line: %s\n", cmd_buffer);
      
      return_value = (tiUINT16)system((const char*)cmd_buffer);
      
      console_printf_terminal("g_tester - Execution result: 0x%x\n", return_value);
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_GENERAL_CMD_RUN_CMD, (UINT8)0x00);
      
      /* Add the result */
      P_BUFFER_ADD_UINT16(p_return_buffer, return_value);
      
      g_tester_send_to_host(return_buffer, 5);
  }
  
  /************************************************************************
  *                        g_tester_process_get_init_table	            *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    : main process only! 			 
  ************************************************************************/
  void g_tester_process_get_init_table(unsigned char *cmd_buffer)
  {
      UINT8 buffer[(1024 * 5)];
      UINT8 *p_return_buffer = buffer;
      UINT32 length;
      
      UINT8 init_table_file_name_length = *(cmd_buffer + 0); 
      unsigned char *init_table_file_name = (cmd_buffer + 1); 
      
      UINT8 error_code = G_TESTER_GWSI_GET_INIT_TABLE_NO_ERROR;
      
      int FileDescriptor;
      
      if (init_table_file_name_length == 0)
      {
          console_printf_terminal("g_tester, g_tester_process_get_init_table, Error - no file name!\n");
          
          error_code = G_TESTER_GWSI_GET_INIT_TABLE_ERROR_NO_FILE_NAME;
      }
      else
      {
          /* Get the Init buffer from the driver */
          TI_GWSIGetInitTable(g_id_adapter, (tiUINT32 *)&buffer[0]);
          
          /* The first 4 bytes are the buffer length */
          length = *(UINT32 *)&buffer[0];
          
          if (length > 0)
          {
              FileDescriptor = open((const char*)init_table_file_name, O_CREAT | O_WRONLY);
              
              if (FileDescriptor != -1)
              {
                  if (write(FileDescriptor, buffer, length + sizeof(UINT32) ) != length + sizeof(UINT32))
                  {
                      console_printf_terminal("g_tester, g_tester_process_get_init_table, Error writing to file (%d)\n", errno);
                      
                      error_code = G_TESTER_GWSI_GET_INIT_TABLE_ERROR_WRITING_TO_FILE;
                  }
                  else
                  {
                      console_printf_terminal("g_tester, g_tester_process_get_init_table, Written 0x%x bytes to %s\n\n", (length - 4), init_table_file_name);
                  }
                  
                  close(FileDescriptor);
              }
              else
              {
                  console_printf_terminal("g_tester, g_tester_process_get_init_table, Error creating %s (%d)\n", init_table_file_name, errno);
                  
                  error_code = G_TESTER_GWSI_GET_INIT_TABLE_ERROR_CREATING_FILE;
              }
          }
          else
          {
              console_printf_terminal("g_tester, g_tester_process_get_init_table, Error - driver init table not availble!\n");
              
              error_code = G_TESTER_GWSI_GET_INIT_TABLE_ERROR_TABLE_NOT_AVAIL;
          }
      }
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_GENERAL_CMD_GET_INIT_T, error_code);
      
      g_tester_send_to_host(buffer, 3);
  }
  
  /************************************************************************
  *                        g_tester_cmd_status                           *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    : main process only! 			 
  ************************************************************************/
  void g_tester_cmd_status()
  {
  /***************************
  Return buffer structure 
  
    Bytes 0-1  : OpCode (0x0003)
    Byte  2	   : Command Status
    Byte  3	   : Status (0/1/2)
    Bytes 4-9  : MAC Address
    Byte  10   : SSID length
    Bytes 11-42: SSID 
    Bytes 43-48: BSSID
    Byte  49:  : Channel
      ***************************/
      
      tiUINT8 return_buffer[50];
      tiUINT8 *p_return_buffer = return_buffer;
      
      tiINT32 res;
      tiUINT32 data;
      OS_802_11_BSSID_EX	bssid_ex;
      OS_802_11_MAC_ADDRESS mac_address;
      OS_802_11_SSID ssid;
      
      /* Set the header to dummy values */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_DRIVER__STATUS, 0x00);
      
      /* Get the Driver status (Running / Stop) */
      res = TI_WLAN_IsDriverRun(g_id_adapter, (tiBOOL *)&data);
      
      if (res == 0)
      {
          /* Insert the status to the return buffer */
          P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)data);
          
          /* Get the MAC address */
          res = TI_GetCurrentAddress(g_id_adapter, &mac_address);
          
          if (res == 0)
          {
              P_BUFFER_ADD_DATA(p_return_buffer, &mac_address, sizeof(mac_address));
              
              /* Get the SSID */
              res = TI_GetCurrentSSID(g_id_adapter, &ssid);
              
              if (res == 0)
              {
                  /* killme!!!*/
                  /*if (isJunkSSID((void *)&ssid))*/
                  /*{*/
                  /* Put '0' at the length field */
                  /*	ssid.SsidLength = 0;*/
                  /*}*/
                  
                  /* Add ssid length */
                  P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)ssid.SsidLength);
                  
                  /* Add ssid */
                  P_BUFFER_ADD_DATA(p_return_buffer, ssid.Ssid, sizeof(ssid.Ssid));
                  
                  /* Get the BSSID */
                  res = TI_GetSelectedBSSIDInfo(g_id_adapter, &bssid_ex);
                  
                  if (res == 0)
                  {
                      /* Add the BSSID */
                      P_BUFFER_ADD_DATA(p_return_buffer, &bssid_ex.MacAddress, sizeof(bssid_ex.MacAddress));
                      
                      /* Get the Channel */
                      res = TI_GetCurrentChannel(g_id_adapter, &data);
                      
                      if (res == 0)
                      {
                          /* Add channel */
                          P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)data);
                      }
                  }
              }
          }
      }
      
      p_return_buffer = return_buffer;
      
      /* Set the G_Tester result value */
      res = (res == 0) ? G_TESTER_STATUS_OK : G_TESTER_STATUS_ERROR;
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_DRIVER__STATUS, (UINT8)res);
      
      g_tester_send_to_host(return_buffer, 50);
  }	
  
  /************************************************************************
  *                        g_tester_cmd_status                           *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    : main process only! 			 
  ************************************************************************/
  void g_tester_bssid_list()
  {
  /***************************
  Return buffer structure (for header):
  
    Bytes 0-1  : OpCode (0x0004)
    Byte  2	   : Command Status
    Byte  3	   : Number of elements 
      ***************************/
      
      /***************************
      Return buffer structure (for each element):
      
        Bytes 0-1	: OpCode (0x0005)
        Byte  2	   :  Command Status
        Bytes 3-8   : MAC Address
        Byte  9     : Privacy
        Bytes 10-13 : RSSI
        Byte  14    : Infra mode
        Byte  15    : Channel
        Bytes 16-19 : Qos
        Byte  20	: SSID length
        Bytes 21-52 : SSID 
      ***************************/
      
      tiUINT8 return_buffer[55];
      tiUINT8 *p_return_buffer = return_buffer;
      
      OS_802_11_BSSID_LIST_EX *list;/* = (OS_802_11_BSSID_LIST_EX *) data; */
      OS_802_11_BSSID_EX *bssid;
      tiUINT32 number_items;
      tiINT32 res;
      tiUINT8 index;
      tiUINT32 Qos = 0;
      
      console_printf_terminal("g_tester,  g_tester_bssid_list()\n");
      
      res = TI_GetBSSIDList(g_id_adapter, &list);
      if( res || !list )
      {
          /*************************/
          /* Error retrieving data */
          /***********************/
          
          /* Set the header */
          P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST, G_TESTER_STATUS_ERROR);
          
          /* Set dummy UINT8 */
          P_BUFFER_ADD_UINT8(p_return_buffer, 0x00);
          
          g_tester_send_to_host(return_buffer, 4);
      }
      else
      {
          bssid = &list->Bssid[0];
          number_items = list->NumberOfItems;
          
          /*********************/
          /* Header structure */
          /*******************/
          
          /* Set the header */
          P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST, G_TESTER_STATUS_OK);
          
          /* Set dummy UINT8 */
          P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)number_items);
          
          g_tester_send_to_host(return_buffer, 4);
          
          /***********************/
          /* Elements structure */
          /*********************/
          
          for (index = 0; index < number_items; index++)
          {
              p_return_buffer = return_buffer;
              
              /* Set the header */
              P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, (G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST + 1), G_TESTER_STATUS_OK);
              
              P_BUFFER_ADD_DATA(p_return_buffer, bssid->MacAddress, sizeof(bssid->MacAddress));
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)bssid->Privacy);
              
              P_BUFFER_ADD_UINT32(p_return_buffer, bssid->Rssi);
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)bssid->InfrastructureMode);
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)Freq2Chan(bssid->Configuration.Union.channel));
              
              Qos = parseBssidIe(bssid);
              P_BUFFER_ADD_UINT32(p_return_buffer, (tiUINT32)Qos);
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)bssid->Ssid.SsidLength);
              
              P_BUFFER_ADD_DATA(p_return_buffer, bssid->Ssid.Ssid, sizeof(bssid->Ssid.Ssid));
              
              g_tester_send_to_host(return_buffer, 53);
              
              /* Move to the next bssid */
              bssid = (OS_802_11_BSSID_EX *) (((char *) bssid) + bssid->Length);
          }
          
          free(list);
      }
}

/************************************************************************
*                        g_tester_register_event                       *
************************************************************************
DESCRIPTION: 

  CONTEXT    :  			 
  ************************************************************************/
  void g_tester_register_event(tiUINT16 event_mask)
  {
      int index;
      ConParm_t param;
      
      console_printf_terminal("g_tester,  g_tester_register_event (Mask = 0x%x)\n", event_mask);
      
      /* Go over the mask bits */
      for (index = 0; index < 16; index++)
      {
          if ((event_mask & (1 << index)) != (tiUINT16)0x0000)
          {
              param.value = index;
              cmd_events_register(&param, 1);
          }
      }
  }
  
  /************************************************************************
  *                        g_tester_unregister_event                     *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_unregister_event(tiUINT16 event_mask)
  {
      int index;
      ConParm_t param;
      
      console_printf_terminal("g_tester,  g_tester_unregister_event (Mask = 0x%x)\n", event_mask);
      
      /* Go over the mask bits */
      for (index = 0; index < 16; index++)
      {
          if ((event_mask & (1 << index)) != (tiUINT16)0x0000)
          {
              param.value = index;
              cmd_events_unregister(&param, 1);
          }
      }
  }
  
  /************************************************************************
  *                        g_tester_cmd_debug_driver_print               *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_cmd_debug_driver_print(unsigned char *cmd_buffer)
  {
#ifdef TI_DBG
      
      tiUINT32 *buf = (tiUINT32 *)cmd_buffer;
      
      ConParm_t param[2];
      
      param[0].value = buf[0];
      param[1].value = buf[1];
      
      console_printf_terminal("DEBUG: values: (0x%x, 0x%x)\n", param[0].value, param[1].value);
      
      cmd_debug_driver_print((ConParm_t *)&param, 2);
      
#endif
  }
  
  /************************************************************************
  *                        g_tester_receive_event                        *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_receive_event(unsigned char event_index)
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1  : OpCode (0x1050)
    Byte  2	   : Command Status
    Byte  3	   : Event ID 
      ***************************/
      
      tiUINT8 return_buffer[55];
      tiUINT8 *p_return_buffer = return_buffer;
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, 0x1050, G_TESTER_STATUS_OK);
      
      /* Set event index */
      P_BUFFER_ADD_UINT8(p_return_buffer, event_index);
      
      g_tester_send_to_host(return_buffer, 4);
  }
  
  /************************************************************************
  *                        g_tester_send_received_event                  *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_send_received_event(unsigned int cmd_op, tiUINT8 status)
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1  : OpCode (0xFFFF)
    Byte  2-3  : Command Opcode
      ***************************/
      
      tiUINT8 return_buffer[5];
      tiUINT8 *p_return_buffer = return_buffer;
      
      /* Add the event opcode */
      P_BUFFER_ADD_UINT16(p_return_buffer, 0x1000);
      
      /* Add the command opcode */
      P_BUFFER_ADD_UINT16(p_return_buffer, cmd_op);
      
      /* Add the command opcode */
      P_BUFFER_ADD_UINT8(p_return_buffer, status);
      
      g_tester_send_to_host(return_buffer, 5);
  }
  
  /************************************************************************
  *                        g_tester_process_gwsi_init_cmd                *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_process_gwsi_init_cmd(unsigned char *cmd_buffer)
  {
      char *firmware_file_name;
      unsigned char firmware_file_name_length;
      char *e2prom_file_name;
      unsigned char e2prom_file_name_length;
      
      unsigned char *init_buffer = NULL; 
      UINT32 init_buffer_size;
      
      unsigned char host_event_buffer[3];
      unsigned char *p_host_event_buffer = host_event_buffer;
      
      FILE *firmware_file = NULL;
      FILE *e2prom_file = NULL;
      UINT32 firmware_image_length = 0;
      UINT32 e2prom_image_length = 0;
      
      unsigned char error_code = G_TESTER_GWSI_INIT_NO_ERROR;
      
      /* Prepare the parameters */
      firmware_file_name_length = *(cmd_buffer + 0); 
      firmware_file_name = (char*)(cmd_buffer + 1); 
      e2prom_file_name_length = *(cmd_buffer + firmware_file_name_length + 2); 
      e2prom_file_name = (char*)(cmd_buffer + firmware_file_name_length + 3); 
      
      console_printf_terminal("g_tester, DEBUG:0x%x, 0x%x!\n", firmware_image_length, e2prom_file_name_length);
      
      /***************************/
      /* Open the firmware file */
      /*************************/
      
      if ((firmware_file = fopen(firmware_file_name, "r")) != NULL)
      {
          /* Get firmware file length */
          if (fseek(firmware_file, 0, SEEK_END) == 0)
          {
              firmware_image_length = ftell(firmware_file);
              rewind(firmware_file);
              
              console_printf_terminal("g_tester, GWSI Init, Firmeware image file is %s, size = 0x%x!\n", firmware_file_name, firmware_image_length);
          }
          else
          {
              console_printf_terminal("g_tester, Error retriving firmware file length!\n");
              
              error_code = G_TESTER_GWSI_INIT_ERROR_READING_FW_FILE_LENGTH;
          }
      }
      else
      {
          console_printf_terminal("g_tester, Error opening firmware file!\n");
          
          error_code = G_TESTER_GWSI_INIT_ERROR_OPENING_FW_FILE;
      }
      
      /*************************/
      /* Open the e2prom file */
      /***********************/
      
      console_printf_terminal("DEBUG The length is : %d\n", e2prom_file_name_length);
      
      if ((error_code == G_TESTER_GWSI_INIT_NO_ERROR) && (e2prom_file_name_length > 0))
      {
          /* Open the e2prom file */
          if ((e2prom_file = fopen(e2prom_file_name, "r")) != NULL)
          {
              /* Get firmware file length */
              if (fseek(e2prom_file, 0, SEEK_END) == 0)
              {
                  e2prom_image_length = ftell(e2prom_file);
                  rewind(e2prom_file);
                  
                  console_printf_terminal("g_tester, GWSI Init, E2prom image file is %s, size = 0x%x!\n", e2prom_file_name, e2prom_image_length);
              }
              else
              {
                  console_printf_terminal("g_tester, Error retrieving e2prom file length!\n");
                  
                  error_code = G_TESTER_GWSI_INIT_ERROR_READING_EP_FILE_LENGTH;
              }
          }
          else
          {
              console_printf_terminal("g_tester, Error opening e2prom file!\n");
              
              error_code = G_TESTER_GWSI_INIT_ERROR_OPENING_EP_FILE;
          }
      }
      
      /****************************/
      /* Prepare the init struct */
      /**************************/
      
      if (error_code == G_TESTER_GWSI_INIT_NO_ERROR)
      {
          init_buffer_size = 16 + firmware_image_length + e2prom_image_length;
          
          init_buffer = malloc(init_buffer_size);
          
          if (init_buffer != NULL)
          {
              /* Set the GWSI tester command parameters */
              P_BUFFER_ADD_UINT16(init_buffer, G_TESTER_GWSI_CMD_INITIALIZE);	/* Opcode */
              P_BUFFER_ADD_UINT32(init_buffer, init_buffer_size);				/* Length */
              P_BUFFER_ADD_UINT16(init_buffer, 0x0000);							/* Align bytes*/
              P_BUFFER_ADD_UINT32(init_buffer, firmware_image_length);
              P_BUFFER_ADD_UINT32(init_buffer, e2prom_image_length);
              
              init_buffer -= (16); 
              
              /* Read the firmware image */
              if (fread(init_buffer + (16), 1, firmware_image_length, firmware_file) == firmware_image_length)
              {
                  if (e2prom_image_length)
                  {
                      /* Read the e2prom image */
                      if (fread(init_buffer + (16) + firmware_image_length, 1, e2prom_image_length, e2prom_file) != e2prom_image_length)
                      {
                          console_printf_terminal("g_tester, GWSI Init, Error reading e2prom image!\n");
                          
                          error_code = G_TESTER_GWSI_INIT_ERROR_READING_EP_FILE;
                      }
                  }
                  
                  if (error_code == G_TESTER_GWSI_INIT_NO_ERROR)
                  {
                      console_printf_terminal("g_tester, GWSI Init, Sending command to driver (size = 0x%x)!\n", init_buffer_size);
                      
                      /* Send the command */
                      TI_GWSIInitialize(g_id_adapter, (tiUINT32 *)init_buffer);
                  }
              }
              else
              {
                  console_printf_terminal("g_tester, GWSI Init, Error reading firmware image!\n");
                  
                  error_code = G_TESTER_GWSI_INIT_ERROR_READING_FW_FILE;
              }
          }
          else
          {
              console_printf_terminal("g_tester, GWSI Init, Error allocating memory for init buffer!\n");
              
              error_code = G_TESTER_GWSI_INIT_ERROR_ALLOCATION_MEMORY;
          }
      } 
      
      /************************************/
      /* Fall-back -> free all resources */
      /**********************************/
      
      if (firmware_file)
      {
          fclose(firmware_file);
      }
      
      if (e2prom_file)
      {
          fclose(e2prom_file);
      }
      
      if (init_buffer)
      {
          free(init_buffer);
      }		
      
      /****************************/
      /* Send result to the host */
      /**************************/
      
      if (error_code != G_TESTER_GWSI_INIT_NO_ERROR)
      {
          /*************************************************/
          /* Send event with error indication to the host */
          /***********************************************/
          
          P_BUFFER_ADD_HDR_PARAMS(p_host_event_buffer, G_TESTER_GWSI_CMD_INITIALIZE, error_code);
          
          g_tester_send_to_host(host_event_buffer, 3);
      }
}

/************************************************************************
*                        g_tester_process_gwsi_config_cmd              *
************************************************************************
DESCRIPTION: 

  CONTEXT    :  			 
  ************************************************************************/
  void g_tester_process_gwsi_config_cmd(unsigned char *cmd_buffer)
  {
      char *init_file_name;
      unsigned char init_file_name_length;
      
      unsigned char host_event_buffer[3];
      unsigned char *p_host_event_buffer = host_event_buffer;
      
      int init_file_descriptor;
      struct stat file_status_record;
      
      UINT8 *buffer = NULL;
      
      UINT8 error_code = G_TESTER_GWSI_CONFIG_NO_ERROR;
      
      /* Prepare the parameters */
      init_file_name_length = *(cmd_buffer + 0); 
      init_file_name = (char*)(cmd_buffer + 1); 
      
      init_file_descriptor = open(init_file_name, O_RDONLY);
      
      if (init_file_descriptor != -1)
      {
          if (fstat(init_file_descriptor, &file_status_record) != -1)
          {
              buffer = malloc(file_status_record.st_size + (sizeof(UINT16) * 2));
              
              if (buffer != NULL)
              {
                  int temp;
                  if ((temp = read(init_file_descriptor, buffer + (sizeof(UINT16) * 2), file_status_record.st_size)) == file_status_record.st_size)
                  {
                      console_printf_terminal("g_tester, GWSI_Config, Sending config request to driver (file = %s, buffer size = 0x%x)\n", init_file_name, file_status_record.st_size + (sizeof(UINT16) * 2));
                      
                      P_BUFFER_ADD_UINT16(buffer, G_TESTER_GWSI_CMD_CONFIG);
                      P_BUFFER_ADD_UINT16(buffer, file_status_record.st_size);
                      buffer -= (sizeof(UINT16) * 2);
                      
                      /* Send the command to the driver */
                      TI_GWSIConfig(g_id_adapter, (tiUINT32 *)buffer);
                  }
                  else
                  {
                      console_printf_terminal("g_tester, GWSI_Config, Error reading from file (%d)\n", errno);
                      error_code = G_TESTER_GWSI_CONFIG_ERROR_READING_FILE; 
                  }
              }
              else
              {
                  console_printf_terminal("g_tester, GWSI_Config, Error allocating memory (%d)\n", errno);
                  error_code = G_TESTER_GWSI_CONFIG_ERROR_ALLOCATING_MEMORY;
              }
          }
          else
          {
              console_printf_terminal("g_tester, GWSI_Config, Error retriving file size (%d)\n", errno);
              error_code = G_TESTER_GWSI_CONFIG_ERROR_GETIING_FILE_SIZE;
          }
      }
      else
      {
          console_printf_terminal("g_tester, GWSI_Config, Error opening file (%d)\n", errno);
          error_code = G_TESTER_GWSI_CONFIG_ERROR_OPENING_FILE;
      }
      
      /*******************/
      /* Free resources */
      /*****************/
      
      if (init_file_descriptor != -1)
      {
          close(init_file_descriptor);
      }
      
      if (buffer != NULL)
      {
          free(buffer);
      }
      
      /****************************/
      /* Send result to the host */
      /**************************/
      
      if (error_code != G_TESTER_GWSI_CONFIG_NO_ERROR)
      {
          P_BUFFER_ADD_HDR_PARAMS(p_host_event_buffer, G_TESTER_GWSI_CMD_CONFIG, error_code);
          
          g_tester_send_to_host(host_event_buffer, 3);
      }
  }
  
  /************************************************************************
  *                        g_tester_process_gwsi_release_cmd                     *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_process_gwsi_release_cmd(unsigned char *cmd_buffer)
  {
      
      tiUINT8 return_buffer[2];
      tiUINT8 *p_return_buffer = return_buffer;
      
      /* Add the event opcode */
      P_BUFFER_ADD_UINT16(p_return_buffer, G_TESTER_GWSI_CMD_RELEASE);
      
      console_printf_terminal("g_tester, GWSI_Release, Sending release to driver.\n");
      
      /* Send the command to the driver */
      TI_GWSIRelease(g_id_adapter, (tiUINT32 *)return_buffer);
  }
  
  
  /************************************************************************
  *                        g_tester_process_gwsi_cmd                     *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_process_gwsi_cmd(unsigned char *cmd_buffer)
  {
      TI_GWSICommand(g_id_adapter, (tiUINT32 *)cmd_buffer);
  }
  
  /************************************************************************
  *                        g_tester_process_gwsi_cmd                     *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_cmd_get_version()
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1  : OpCode (0x1050)
    Byte  2	   : Command Status
    Byte  3-6  : Driver Version
    Byte  7-10 : FW Version
    Byte  11-14: HW Version
    Byte  15-18: NVM Version
      ***************************/
      
      tiUINT32 ret;
      tiUINT8 return_buffer[19];
      tiUINT8 *p_return_buffer = return_buffer;
      
      TIWLN_VERSION_EX data;
      
      console_printf_terminal("g_tester,  g_tester_cmd_get_version.\n");
      
      ret = (tiUINT8)TI_GetDriverVersion(g_id_adapter, &data );
      
      if (ret == 0)
      {
          /* Set the header */
          P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_ROOT__ABOUT, G_TESTER_STATUS_OK);
          
          /* ADD the versions */
          P_BUFFER_ADD_UINT32(p_return_buffer, (UINT32)(*(UINT32 *)&data.DrvVersion));
          P_BUFFER_ADD_UINT32(p_return_buffer, (UINT32)(*(UINT32 *)&data.FWVersion));
          P_BUFFER_ADD_UINT32(p_return_buffer, (UINT32)(*(UINT32 *)&data.HWVersion));
          P_BUFFER_ADD_UINT32(p_return_buffer, (UINT32)(*(UINT32 *)&data.NVVersion));
      }
      else
      {
          /* Set the header */
          P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_ROOT__ABOUT, G_TESTER_STATUS_ERROR);
      }
      
      g_tester_send_to_host(return_buffer, 19);
  }
  
  
  /************************************************************************
  *                        g_tester_get_advanced_statistics_report       *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_set_rate(tiUINT8 rate_index)
  {
      ConParm_t param;
      char rate_1mbps_rate_str[5] = "1";
      char rate_2mbps_rate_str[5] = "2";
      char rate_5_5mbps_rate_str[5] = "5.5";
      char rate_11mbps_rate_str[5] = "11";
      char rate_22mbps_rate_str[5] = "22";
      
      switch(rate_index)
      {
      case 0:
          param.value = (tiUINT32)rate_1mbps_rate_str;
          break;
          
      case 1:
          param.value = (tiUINT32)rate_2mbps_rate_str;
          break;
          
      case 2:
          param.value = (tiUINT32)rate_5_5mbps_rate_str;
          break;
          
      case 3:
          param.value = (tiUINT32)rate_11mbps_rate_str;
          break;
          
      case 4:
          param.value = (tiUINT32)rate_22mbps_rate_str;
          break;
          
      default:
          param.value = (tiUINT32)rate_1mbps_rate_str;
          break;
      }
      
      cmd_modify_rate(&param, 1);
  }
  
  /************************************************************************
  *                        g_tester_wep_add_key					        *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  void g_tester_wep_add_key(unsigned char *cmd_buffer)
  {
      UINT8 key_length;
      UINT8 key_type;
      ConParm_t key_params[4];
      UINT8 hex_str[] = "hex";
      UINT8 text_str[] = "text";
      
      /* Zero the variables (because i use 8 bit in 32 bit variables) */
      key_params[1].value = 0;
      key_params[2].value = 0;
      
      /* Read parameters */
      key_length = cmd_buffer[65];		/* Buffer length */
      key_type = cmd_buffer[68];			/* Key type (hex = 0, text = 1) */
      
      /* Prepare parameters for the command */
      key_params[0].value = (tiUINT32)&cmd_buffer[0];
      cmd_buffer[key_length] = 0;
      
      key_params[1].value = (tiUINT32)cmd_buffer[66];
      
      key_params[2].value = (tiUINT32)cmd_buffer[67];
      
      if (key_type == 0)
      {
          key_params[3].value = (tiUINT32)&hex_str[0];
      }
      else
      {
          key_params[3].value = (tiUINT32)&text_str[0];
      }
      
      /* Call the addkey command */
      cmd_privacy_addkey(&key_params[0], 4);
  }
  
  /************************************************************************
  *                        g_tester_gwsi_event_handler			        *
  ************************************************************************
  DESCRIPTION: 
  
    CONTEXT    :  			 
  ************************************************************************/
  int g_tester_gwsi_event_handler(IPC_EV_DATA* pData)
  {
      console_printf_terminal("g_tester,  g_tester_gwsi_event_handler. (Length = %d)\n", pData->uBufferSize);
      
      g_tester_send_to_host(pData->uBuffer, pData->uBufferSize);
      
      return 0;
  }
  
  /************************************************************************
  *                        g_tester_gwsi_event_handler                  *
  ************************************************************************
  DESCRIPTION: Read the register value and send it back by event.
  
  ************************************************************************/
  void g_tester_cmd_plt_register_read(UINT32 uiRegAddress)
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1  : OpCode (0x0173)
    Byte  2    : Command Status
    Byte  3-6  : Register value
      ***************************/
      
      tiUINT8 return_buffer[19];
      tiUINT8 *p_return_buffer = return_buffer;
      
      UINT32 uiRegisterValue;
      tiUINT32 status = TI_PLT_ReadRegister(g_id_adapter, uiRegAddress, &uiRegisterValue);
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT__REGISTER__READ, (UINT8)status);
      
      /* Add the result */
      P_BUFFER_ADD_UINT32(p_return_buffer, uiRegisterValue);
      
      g_tester_send_to_host(return_buffer, 7);
  }
  
  /************************************************************************
  *                        g_tester_gwsi_event_handler                  *
  ****** ******************************************************************
  DESCRIPTION: Read the PLT RX PER .
  
  ************************************************************************/
  void g_tester_cmd_plt_RxPer_GetResults()
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1   : OpCode (0x0178)
    Byte  2     : Command Status
    Byte  3-6   : FCSErrorCount
    Byte  7-10  : TotalFrameCount
    Byte  11-14 : PLCPFrameCount
    Byte  15-18 : SeqNumMissCount
      ***************************/
      
      tiUINT8 return_buffer[20];
      tiUINT8 *p_return_buffer = return_buffer;
      
      PltRxPer_t PltRxPer; 
      tiUINT32 status = TI_PLT_RxPerGetResults(g_id_adapter, &PltRxPer);
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT__RX_PER__GET_RESULTS, (UINT8)status);
      
      /* Add the result */
      P_BUFFER_ADD_UINT32(p_return_buffer, PltRxPer.FCSErrorCount);
      P_BUFFER_ADD_UINT32(p_return_buffer, PltRxPer.TotalFrameCount);
      P_BUFFER_ADD_UINT32(p_return_buffer, PltRxPer.PLCPErrorCount);
      P_BUFFER_ADD_UINT32(p_return_buffer, PltRxPer.SeqNumMissCount);
      
      g_tester_send_to_host(return_buffer, p_return_buffer - return_buffer);
  }
  
  /************************************************************************
  *                        g_tester_cmd_plt_mib_read                  *
  ****** ******************************************************************
  DESCRIPTION: Handle the reading of PLT over CLI MIBs .
  
  ************************************************************************/
  void g_tester_cmd_plt_mib_read(unsigned char *cmd_buffer)
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1  : OpCode (0x017C)
    Byte  2    : Command Status
    Byte  3-6  : aMib 
    Byte  7-10 : Mib Length
    Bye   11-  : MIb data
      ***************************/
      
      PLT_MIB_t* pMib = (PLT_MIB_t*)cmd_buffer;
      
      unsigned char return_buffer[3 + sizeof(PLT_MIB_t)];
      unsigned char* p_return_buffer = return_buffer;
      unsigned char* pReturnMibBuffer = return_buffer + 3;
      PLT_MIB_t* pReturnMib = (PLT_MIB_t*)pReturnMibBuffer;
      tiUINT32 status;
      int PacketLength;
      
      pReturnMib->aMib =  pMib->aMib;
      pReturnMib->Length = pMib->Length;
      memcpy(&pReturnMib->aData, &pMib->aData, pMib->Length);
      
      status = TI_PLT_ReadMIB(g_id_adapter, pReturnMib);
      
      /* Set the event header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT__MIB__READ, (UINT8)status);
      
      /* Set the MIB data */ 
      PacketLength = 11 + pReturnMib->Length;
      g_tester_send_to_host(return_buffer, PacketLength);    
  }
  
  /************************************************************************
  *                        g_tester_cmd_plt_mib_write                  *
  ****** ******************************************************************
  DESCRIPTION: Handle the writing of PLT over CLI MIBs .
  
  ************************************************************************/
  void g_tester_cmd_plt_mib_write(unsigned char *cmd_buffer)
  {
      PLT_MIB_t* pMib = (PLT_MIB_t*)cmd_buffer;
      TI_PLT_WriteMIB(g_id_adapter, pMib);
  }
  
  
  
  
  /************************************************************************
  *                        g_tester_cmd_get_defaultWEPKey                  *
  ****** ******************************************************************
  DESCRIPTION: Get back the default WEP key .
  
  ************************************************************************/
  void g_tester_cmd_get_defaultWEPKey(void)
  {
  /***************************
  Return buffer structure :
  
    Bytes 0-1  : OpCode (0x117e)
    Byte  2    : Command Status
    Byte  3-6  : Default WEP key ID
      ***************************/
      
      tiUINT8 return_buffer[6];
      tiUINT8 *p_return_buffer = return_buffer;
      tiUINT32 WepKeyId;
      tiUINT32 status = TI_GetDefaultWepKey(g_id_adapter, &WepKeyId);
      
      /* Set the header */
      P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PRIVACY__WEP__GET, (UINT8)status);
      
      /* Add the result */
      P_BUFFER_ADD_UINT32(p_return_buffer, WepKeyId);
      
      g_tester_send_to_host(return_buffer, 5);
  }
  
  /************************************************************************
  *                        g_tester_gwsi_event_handler			        *
  ************************************************************************
  DESCRIPTION: Send the roaming candidates table to the GTester
  
  ************************************************************************/
  void g_tester_Roaming_candidates_list(void)
  {
  /***************************
  Return buffer structure (for header):
  
    Bytes 0-1  : OpCode (0x0004)
    Byte  2	   : Command Status
    Byte  3	   : Number of elements 
      ***************************/
      
      /***************************
      Return buffer structure (for each element):
      
        Bytes 0-1	: OpCode (0x113c)
        Byte  2	    : Command Status
        Bytes 3-8   : MAC Address
        Byte  9     : Band
        Byte  10    : RSSI
        Byte  11    : Channel
        Byte  12    : Neighbor
      ***************************/
      tiUINT8 return_buffer[15];
      tiUINT8 *p_return_buffer = return_buffer;
      bssList_t bssList;
      tiINT32 res;
      tiUINT8 index;
      
      
      /* get list */
      res = TI_GetScanBssList( g_id_adapter, &bssList);
      if( res != TI_RESULT_OK)
      {
          /*************************/
          /* Error retrieving data */
          /***********************/
          
          /* Set the header */
          P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST, res);
          
          /* Set dummy UINT8 */
          P_BUFFER_ADD_UINT8(p_return_buffer, 0x00);
          
          g_tester_send_to_host(return_buffer, 4);
      }
      else
      {
          /*********************/
          /* Header structure */
          /*******************/
          
          /* Set the header */
          P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST, res);
          
          /* Set dummy UINT8 */
          P_BUFFER_ADD_UINT8(p_return_buffer, bssList.numOfEntries);
          
          g_tester_send_to_host(return_buffer, 4);
          
          /***********************/
          /* Elements structure */
          /*********************/
          
          for (index = 0; index < bssList.numOfEntries; index++)
          {
              p_return_buffer = return_buffer;
              
              /* Set the header */
              P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BSSLIST, res);
              
              P_BUFFER_ADD_DATA(p_return_buffer, bssList.BSSList[index].BSSID.addr , sizeof(bssList.BSSList[index].BSSID));
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)bssList.BSSList[index].band);
              
              P_BUFFER_ADD_UINT8(p_return_buffer, bssList.BSSList[index].RSSI-256);  /*convert to negative number*/
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)bssList.BSSList[index].channel);
              
              P_BUFFER_ADD_UINT8(p_return_buffer, (tiUINT8)bssList.BSSList[index].bNeighborAP);
              
              g_tester_send_to_host(return_buffer, 13);
              
          }  
      }      
  }
  
  /************************************************************************
  *                        g_tester_scAn__configApp__Display			        *
  ************************************************************************
  DESCRIPTION: Display the application scan configuration
  
  ************************************************************************/
void g_tester_scAn__configApp__Display(void)
{
	/***************************
	Return buffer structure (for each element):

	Bytes 0-1	: OpCode (0x113c)
	Byte  2	    : Command Status
	Bytes 3-15  : SSID(if SSID is 12 characters)
    Byte  16    : Scan type
	Byte  10    : Band
	Byte  11    : Number of prob req
	Byte  12    : rate
    Byte  13    : AC
	Byte  14    : Number of channel 
	
    for every channel:  
    Byte  15    : channel number
	Byte  16-21 : BSSID
    Byte  21-23 : max time
    Byte  23-27 : min time
    Byte  28    : ET event
	Byte  29    : ET frame num
    Byte  30    : Power
    Byte  31-   : Same as 15-30 for every channel  grater then 1
	***************************/
   int i;
   scan_normalChannelEntry_t* pNormalChannel;
   tiUINT8 return_buffer[512];
   tiUINT8 sSSID[33];
   tiUINT8 *p_return_buffer = return_buffer;

	/* Set the header */
	P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_SCAN__CONFIGAPP__DISPLAY, 0);
   
    /* Data */
    /* SSID*/
    memset(sSSID, 0, sizeof(sSSID));
    strcpy((char*)sSSID, (char*)appScanParams.desiredSsid.ssidString);
    memcpy((void*)p_return_buffer, sSSID, sizeof(sSSID));
    p_return_buffer += sizeof(sSSID);


    P_BUFFER_ADD_UINT8(p_return_buffer, appScanParams.scanType);
    P_BUFFER_ADD_UINT8(p_return_buffer, appScanParams.band);
    P_BUFFER_ADD_UINT8(p_return_buffer, appScanParams.probeReqNumber);
    P_BUFFER_ADD_UINT32(p_return_buffer, appScanParams.probeRequestRate);
#ifdef TI_DBG    
    P_BUFFER_ADD_UINT8(p_return_buffer, appScanParams.Tid);
#else
    P_BUFFER_ADD_UINT8(p_return_buffer, 0);
#endif
    P_BUFFER_ADD_UINT8(p_return_buffer,  appScanParams.numOfChannels);
    for ( i = 0; i < appScanParams.numOfChannels; i++ )
    {
      pNormalChannel = &(appScanParams.channelEntry[ i ].normalChannelEntry);  
      P_BUFFER_ADD_UINT8(p_return_buffer, pNormalChannel->channel);
      memcpy((void*)p_return_buffer, (void*)(pNormalChannel->bssId.addr), sizeof(pNormalChannel->bssId.addr));
      p_return_buffer += sizeof(pNormalChannel->bssId);
      P_BUFFER_ADD_UINT32(p_return_buffer, pNormalChannel->maxChannelDwellTime);
      P_BUFFER_ADD_UINT32(p_return_buffer, pNormalChannel->minChannelDwellTime);
      P_BUFFER_ADD_UINT8(p_return_buffer, pNormalChannel->earlyTerminationEvent);
      P_BUFFER_ADD_UINT8(p_return_buffer, pNormalChannel->ETMaxNumOfAPframes);
      P_BUFFER_ADD_UINT8(p_return_buffer, pNormalChannel->txPowerDbm);
    }
	g_tester_send_to_host(return_buffer, p_return_buffer-return_buffer);
}

/*********************************************************************
*                        g_tester_plt_calibration_get_nvs_buffer	   *
************************************************************************
DESCRIPTION: Display the NVS update buffers
************************************************************************/
void g_tester_plt_calibration_get_nvs_buffer()
{
   tiUINT8 return_buffer[sizeof(PltNvsResultsBuffer_t) + 4];
   tiUINT8 *p_return_buffer = return_buffer;
   PltNvsResultsBuffer_t PltNvsResultsBuffer;
   int i;
   tiUINT32 status;
   tiUINT32 DataSize;
 
   memset(&PltNvsResultsBuffer, 0, sizeof(PltNvsResultsBuffer));
   status = TI_PLT_RxTxCalNVSUpdateBuffer(g_id_adapter, &PltNvsResultsBuffer);
   P_BUFFER_ADD_HDR_PARAMS(p_return_buffer, G_TESTER_CLI_CMD_PLT__CALIBRATION__GET_NVS_BUFFER, (UINT8)status);
   if (status == OK)
   {
       P_BUFFER_ADD_UINT32(p_return_buffer, PltNvsResultsBuffer.numOfTables);
       if (PltNvsResultsBuffer.numOfTables > NVS_RESULTS_MAX_NUM_OF_TABLES)
       {
           printf("%s:  aData.aLength (%d) > GWSI_PLT_NVS_RESULTS_MAX_NUM_OF_TABLES(%d) \n", __FUNCTION__,
               PltNvsResultsBuffer.numOfTables, NVS_RESULTS_MAX_NUM_OF_TABLES);
       }
       else
       {
           for (i=0; i<PltNvsResultsBuffer.numOfTables; i++)
           {
               DataSize = PltNvsResultsBuffer.tables[i].size;
               P_BUFFER_ADD_UINT16(p_return_buffer, DataSize);
               P_BUFFER_ADD_UINT16(p_return_buffer, PltNvsResultsBuffer.tables[i].offset);
                              
               if (DataSize>NVS_RESULTS_MAX_UPDATE_TABLE_SIZE)
               {
                   printf("%s:  DataSize (%d) > NVS_RESULTS_MAX_UPDATE_TABLE_SIZE(%d) \n", __FUNCTION__,
                       DataSize, NVS_RESULTS_MAX_UPDATE_TABLE_SIZE);
                   continue;
               }
               
               memcpy((PVOID)p_return_buffer, 
                      (PVOID)PltNvsResultsBuffer.tables[i].data,
                      DataSize);
               p_return_buffer += DataSize;
           }                
       }
   }
   	g_tester_send_to_host(return_buffer, p_return_buffer-return_buffer);
}
