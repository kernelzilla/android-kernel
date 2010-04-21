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

#include <stdio.h>
#include <stdlib.h>

#ifdef _WINDOWS
#else /* __LINUX__ */
	#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <linux/in.h>
#include "ipc.h"
#include "wipp_ctrl.h"
#include "eth_utils.h"
#include "dbg_module.h"
#include "g_tester.h"
	#define LINUX_OS_FILES
#endif

#define MAX_CERT_FILE_NAME_LENGTH 32
#define MAX_CERT_PASSWORD_LENGTH 32
#define MAX_CERT_USER_NAME_LENGTH 32
#define MAX_PSK_STRING_LENGTH       64 /* MAX string phrase is 63 chars, but hexa phrase is 64 chars excactly */
#define MIN_PSK_STRING_LENGTH       8  /* MIN string phrase is 8 chars */

#include "paramOut.h"
#include "linux_ioctl_common.h"
#include "tiioctl.h"
#include "console.h"
#include "ticon.h"
#include "cu_cmd.h"

#define WLAN_DEVICE_NAME	    (_T("TIWLNAPI1"))

#ifdef EXC_MODULE_INCLUDED
#include "TI_AdapterEXC.h"
#endif /*EXC_MODULE_INCLUDED*/
#include "TI_AdapterApiC.h"
#include "TI_IPC_Api.h"


extern void osInitTable(initTable_t *InitTable);

void quit_func(void);
void dummy_func(void);

void init_extended_tools(void);
void deinit_extended_tools(void);

char    g_drv_name[IFNAMSIZ + 1];

#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */

#define MAX_HOST_MESSAGE_SIZE	(256)

/************************************************************************
 *                        console_printf_terminal		                *
 ************************************************************************
DESCRIPTION: Sends a text message to host

CONTEXT:  Any process, the 'p_shared_memory' points
		  to the shared memory block
************************************************************************/
int console_printf_terminal(const char *arg_list ,...)
{
#ifndef _WINDOWS /*TRS:WDK slightly different code needed for each OS*/
	va_list ap;
    tiUINT8 msg[MAX_HOST_MESSAGE_SIZE];
	int message_length;

	/***********************/
	/* Format the message */
	/*********************/

	va_start(ap, arg_list);
	message_length = vsnprintf((char*)&msg[0], sizeof(msg), arg_list, ap);

	/*********************************/
	/* Send the message to the host */
	/*******************************/

	return console_send_buffer_to_host(ETHERNET_UTILS_TERMINAL_MODULE_ID, msg, message_length);

#else /* _WINDOWS */
#endif /* _WINDOWS */
}

#ifndef _WINDOWS
#define ETHERNET_PROTOCOL_PREFIX_SIZE	(3)
#define MUX_UART_PROTOCOL_PREFIX_SIZE	(2)
/************************************************************************
 *                        console_send_buffer_to_host                   *
 ************************************************************************
DESCRIPTION: Sends a text message to host

CONTEXT:  Any process, the 'p_shared_memory' points
		  to the shared memory block
************************************************************************/
int console_send_buffer_to_host(tiUINT8 module_inedx, tiUINT8 *buffer, tiUINT16 length)
{
	tiUINT8 output_path = SHARED_MEMORY_OUTPUT_PATH(module_inedx);
	tiUINT8 protocol_header[3];
	int return_size, ret;

	/*****************/
	/* Sanity check */
	/***************/

	if (output_path == OUTPUT_PATH_ETHERNET)
	{
		/***********************/
		/* Output to ethernet */
		/*********************/

		/* Prepare the header */
		protocol_header[0] = 'W';					/* Ethernet protocol Prefix */
		protocol_header[1] = length & 0xFF;			/* Message size (first byte) */
		protocol_header[2] = (length >> 8) & 0xFF;	/* Message size (second byte) */

		/* Send the header */
		ret = write(SHARED_MEMORY_IPC_PIPE(module_inedx), protocol_header, ETHERNET_PROTOCOL_PREFIX_SIZE);
        if( ret <= 0 )
        {
            printf("\tERROR: %s() returned %d (err=%d)\n\n", __FUNCTION__, ret, errno );
            return 0;
        }

		/* Send the message */
		ret = write(SHARED_MEMORY_IPC_PIPE(module_inedx), buffer, length);
        if( ret <= 0 )
        {
            printf("\tERROR1: %s() returned %d (err=%d)\n\n", __FUNCTION__, ret, errno );
            return 0;
        }

		return_size = (ETHERNET_PROTOCOL_PREFIX_SIZE + length);
	}
	else if (output_path == OUTPUT_PATH_MUX_UART)
	{
		/***********************/
		/* OUTPUT to mux UART */
		/*********************/

		/* Write the protocol prefix */
		protocol_header[0] = (28 << 3);
		protocol_header[1] = length;
		fwrite(protocol_header, 1, MUX_UART_PROTOCOL_PREFIX_SIZE, stdout);

		/* Write the message */
		fwrite(buffer, 1, length, stdout);

		return_size = (MUX_UART_PROTOCOL_PREFIX_SIZE + length);
	}
	else
	{
		/*******************/
		/* OUTPUT to UART */
		/*****************/

		/* Use the original printf function */
		return_size = printf("%s", buffer);
	}

	return return_size;
}
#endif /* __LINUX__ */


#ifdef _WINDOWS
#endif /* WINDOWS */

void dummy_func()
{
    console_printf_terminal("1: not implemented yet!!\n");
}

void quit_func(void)
{
    consoleStop();
}

int print_usage(char  *eeprom_file_name, char  *init_file_name, char *firmware_file_name)
{
#ifdef _WINDOWS
#else
    fprintf(stderr, "Usage: ./wlan_cu [driver_name] [options]\n");
#endif
    fprintf(stderr, "   -s <filename>  - run script\n");
    fprintf(stderr, "   -e <filename>  - eeprom image file name. Dft=%s\n", eeprom_file_name);
    fprintf(stderr, "   -i <filename>  - init file name. Dft=%s\n", init_file_name);
    fprintf(stderr, "   -f <filename>  - firmware image file name. Dft=%s\n",firmware_file_name);
    fprintf(stderr, "   -b             - bypass supplicant\n");
#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */
    return 1;
}

int init_console_menu(void)
{
    handle_t h, h1, h2;

    /* -------------------------------------------- Driver -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "Driver",  "Driver start/stop" ) );
        {
            ConParm_t aaa[]  = { { "start_suppl_manager", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Start",  "Start driver", (FuncToken_t) cmd_start_driver, aaa );
        }
            consoleAddToken(h, "stoP",   "Stop driver", (FuncToken_t) cmd_stop_driver, NULL );
            consoleAddToken(h, "stAtus", "Print status", (FuncToken_t) cmd_show_status, NULL );

    /* -------------------------------------------- Connection -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "Connection",  "Connection management" ) );
        consoleAddToken(h, "Bssid_list",  "Bssid_list", (FuncToken_t) cmd_bssid_list, NULL );
        {
            ConParm_t aaa[]  = { { "ssid", 	CON_PARM_STRING | CON_PARM_OPTIONAL, 0, 32, 0 },
								 { "bssid", CON_PARM_STRING | CON_PARM_OPTIONAL, 0, 32, 0 },
								CON_LAST_PARM };

            consoleAddToken(h, "Connect",  "Connect", (FuncToken_t) cmd_connect, aaa );
        }
        consoleAddToken(h, "Disassociate",  "disconnect", (FuncToken_t) cmd_disassociate, NULL );
        consoleAddToken(h, "Status", "Print connection status", (FuncToken_t) cmd_show_status, NULL );
        consoleAddToken(h, "Full_bssid_list",  "Full_bssid_list", (FuncToken_t) cmd_Full_bssid_list, NULL );
        consoleAddToken(h, "full_Primary_bssid",  "Full_Primary_bssid", (FuncToken_t) cmd_FullPrimaryBbssid, NULL );


    /* -------------------------------------------- Management -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "Management",  "Station management" ) );
        {
            ConParm_t aaa[]  = { { "ssid", CON_PARM_LINE | CON_PARM_OPTIONAL, 0, 32, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Ssid",  "Set prefered SSID", (FuncToken_t) cmd_modify_ssid, aaa );
        }
        {
            ConParm_t aaa[]  = { { "channel", /*CON_PARM_RANGE | */CON_PARM_OPTIONAL, 0, 0/*1000*/, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Channel",  "Set the channel", (FuncToken_t) cmd_modify_channel, aaa );
        }
        {
            ConParm_t aaa[]  = { { "tx rate", CON_PARM_STRING | CON_PARM_OPTIONAL, 0, 32, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Rate",  "Get TX data rate in Mbps (1,2,5.5,11,22)", (FuncToken_t) cmd_modify_rate, aaa );
        }
        {
            ConParm_t aaa[]  = { { "BSS_type", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Mode",  "BSS_type", (FuncToken_t) cmd_modify_bss_type, aaa );
        }
        {
            ConParm_t aaa[]  = { { "frag", /*CON_PARM_RANGE | */CON_PARM_OPTIONAL, 0/*256*/, 0/*2346*/, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Frag",  "Set the fragmentation threshold <256..2346>", (FuncToken_t) cmd_modify_frag_threshold, aaa );
        }
        {
            ConParm_t aaa[]  = { { "rts", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "rts",  "Set RTS threshold <0..2347>", (FuncToken_t) cmd_modify_rts_threshold, aaa);
        }
        {
            ConParm_t aaa[]  = { { "preamble", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 1, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "prEamble",  "Set preamble type 1=short 0=long", (FuncToken_t) cmd_modify_preamble, aaa );
        }
        {
            ConParm_t aaa[]  = { { "slot", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "sLot",  "Set short  slot", (FuncToken_t) cmd_modify_short_slot, aaa );
        }
            consoleAddToken(h, "Info",  "Get Selected BSSID Info", (FuncToken_t) cmd_get_selected_bssid_info, NULL );
            consoleAddToken(h, "DriverState",  "Get Driver State", (FuncToken_t) cmd_get_driver_state, NULL );
            consoleAddToken(h, "siGnal",  "Get Current RSSI level", (FuncToken_t) cmd_get_rsii_level, NULL );
			consoleAddToken(h, "snr ratiO",  "Get Current SNR radio", (FuncToken_t) cmd_get_snr_ratio, NULL );


        {
            ConParm_t aaa[]  = { { "Tx power level", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "tX_power_level_table",  "Tx power level", (FuncToken_t) cmd_show_tx_power_level_table, aaa );
            consoleAddToken(h, "tx_power_dBm_div10",  "Tx power level", (FuncToken_t) cmd_tx_power_dbm, aaa );
        }
        consoleAddToken(h, "arP ip addresses table",  "Get ARP IP address table", (FuncToken_t) cmd_get_arpIpTable, NULL);
    consoleAddToken(h, "groUp address table",  "Get Group address table", (FuncToken_t) cmd_get_GroupAddressTable, NULL);


        CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "802_11d_h",  "802_11D_H" ) );
            {
                    ConParm_t aaa[]  = { { "802_11_D", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                    ConParm_t bbb[]  = {
                            { "min DFS channel", CON_PARM_RANGE, 0, 180, 40 },
                            { "max DFS channel", CON_PARM_RANGE, 0, 180, 140 },
                             CON_LAST_PARM};

                    consoleAddToken(h1, "D_enableDisable",  "enableDisable_d", (FuncToken_t) cmd_enableDisable_802_11d, aaa );
                    consoleAddToken(h1, "H_enableDisable",  "enableDisable_h", (FuncToken_t) cmd_enableDisable_802_11h, aaa );
                    consoleAddToken(h1, "d_Country_2_4Ie",  "d_Country_2_4Ie", (FuncToken_t) cmd_d_Country_2_4Ie, aaa );
                    consoleAddToken(h1, "d_cOuntry_5Ie",  "d_Country_5Ie", (FuncToken_t) cmd_d_Country_5Ie, aaa );

                    consoleAddToken(h1, "dfS_range",  "DFS_range", (FuncToken_t) cmd_DFS_range, bbb );

            }


                                /* b-only, g-only, b&g mode, a-only, a&g mode b-plus-mode */
        CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Antenna",  "tx/rx selected antenna" ) );
            {
                ConParm_t aaa[]  = { { "Enable RX diversity", CON_PARM_RANGE, 0, 1, 0 },
                                     { "RX selected antenna", CON_PARM_RANGE, 1, 2, 2 },
                                     { "Enable TX diversity", CON_PARM_RANGE, 0, 1, 0 },
                                     { "TX selected antenna", CON_PARM_RANGE, 1, 2, 2 },
                                     { "Share RX and TX antennas", CON_PARM_RANGE, 0, 1, 1 },
                                     CON_LAST_PARM };
                consoleAddToken(h1, "Diversityparams",  "Set antenna diversity params", (FuncToken_t) cmd_modify_antenna_diversity, aaa );
            }

		CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "beacoN",  "Set Beacon Filter Desired State" ) );
            {
                ConParm_t beaconFilterDesiredState[]  = { { "Set Beacon Desired State", CON_PARM_OPTIONAL, 0, 0, 0 },
                                     CON_LAST_PARM };
                consoleAddToken(h1, "Set Beacon Filter Desired State","Set Beacon Filter Current State", (FuncToken_t) cmd_Beacon_Filter_Set_Desired_State, beaconFilterDesiredState );
				consoleAddToken(h1, "Get Beacon Filter Current State","Get Beacon Filter Current State", (FuncToken_t) cmd_Beacon_Filter_Get_Desired_State, beaconFilterDesiredState );
            }


        CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "adVanced",  "Advanced params" ) );
            {
                ConParm_t aaa[]  = { { "ext_rates", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h1, "Draft",  "Draft", (FuncToken_t) cmd_modify_ext_rates_ie, aaa );
            }
            {
                ConParm_t aaa[]  = { { "rates", CON_PARM_OPTIONAL | CON_PARM_LINE, 0, 120, 0 }, CON_LAST_PARM };
                consoleAddToken(h1, "Supported rates",  "rates", (FuncToken_t) cmd_modify_supported_rates, aaa );
            }
            {
                ConParm_t aaa[]  = { { "CtsToSelf", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 1, 0 }, CON_LAST_PARM };
                consoleAddToken(h1, "Cts to self",  "Enable/Disable CTS to self", (FuncToken_t) cmd_modify_ctsToSelf, aaa );
            }
            CHK_NULL(h2 = (handle_t) consoleAddDirExt( (handle_t) h1, "rx data Filter",  "Rx Data Filter" ) );
                consoleAddToken(h2, "Enable",  "Enable Rx Data Filtering", (FuncToken_t) cmd_enable_rx_data_filters, NULL );
                consoleAddToken(h2, "Disable",  "Enable Rx Data Filtering", (FuncToken_t) cmd_disable_rx_data_filters, NULL );
                {
                    ConParm_t aaa[]  =
                    {
                        { "Offset", CON_PARM_RANGE, 0, 255, 0 },
                        { "Mask", CON_PARM_STRING, 0, 64, 0 },
                        { "Pattern", CON_PARM_STRING, 0, 128, 0 },
                        CON_LAST_PARM
                    };
                    consoleAddToken(h2, "Add",  "Add Rx Data Filter", (FuncToken_t) cmd_add_rx_data_filter, aaa );
                }
                {
                    ConParm_t aaa[]  =
                    {
                        { "Offset", CON_PARM_RANGE, 0, 255, 0 },
                        { "Mask", CON_PARM_STRING, 0, 64, 0 },
                        { "Pattern", CON_PARM_STRING, 0, 128, 0 },
                        CON_LAST_PARM
                    };
                    consoleAddToken(h2, "Remove",  "Remove Rx Data Filter", (FuncToken_t) cmd_remove_rx_data_filter, aaa );
                }
                consoleAddToken(h2, "Statistics",  "Print Rx Data Filtering Statistics", (FuncToken_t) cmd_get_rx_data_filters_statistics, NULL );

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "Show",  "Show params" ) );
        consoleAddToken(h, "Statistics",  "Show statistics", (FuncToken_t) cmd_show_statistics, NULL );
		{
			ConParm_t aaa[]  = { { "Clear stats on read", CON_PARM_OPTIONAL | CON_PARM_RANGE, 0, 1, 0 }, CON_LAST_PARM };
			consoleAddToken(h, "Tx statistics",  "Show tx statistics", (FuncToken_t) cmd_show_tx_statistics, aaa );
		}
		consoleAddToken(h, "Advanced",  "Show advanced params", (FuncToken_t) cmd_show_advanced_params, NULL );

        consoleAddToken(h, "Power consumption",  "Show power consumption statistics", (FuncToken_t) cmd_show_power_consumption_stats, NULL );

        /* -------------------------------------------- Privacy -------------------------------------------- */

        CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "Privacy",  "Privacy configuration" ) );
            {
                ConParm_t aaa[]  = { { "mode", CON_PARM_OPTIONAL, 0, 0, 0 },CON_LAST_PARM };
                consoleAddToken(h, "Authentication",  "Set authentication mode",
                            (FuncToken_t)cmd_privacy_auth, aaa );
            }
            {
                ConParm_t aaa[]  = { { "type", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h, "EAP",  "Set EAP type", (FuncToken_t)cmd_privacy_eap, aaa );
            }
            {
                ConParm_t aaa[]  = { { "type", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h, "encRyption",  "Set Encryption type", (FuncToken_t)cmd_privacy_encrypt, aaa);
            }

            {
                ConParm_t aaa[]  = { { "type", 0, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h, "Keytype",  "Set key type", (FuncToken_t) cmd_privacy_key_type, aaa );
            }

            {
                ConParm_t aaa[]  = { { "mode", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h, "Mixedmode",  "Set mixed mode", (FuncToken_t) cmd_privacy_mixed_mode, aaa );
            }

            {
                ConParm_t aaa[]  = {
                    { "User:", CON_PARM_STRING, 0, MAX_CERT_USER_NAME_LENGTH, 0 },
                    { "Password:", CON_PARM_STRING | CON_PARM_OPTIONAL, 0, MAX_CERT_PASSWORD_LENGTH , 0 },
                    CON_LAST_PARM };
                consoleAddToken(h, "Credentials",  "Set Credentials ", (FuncToken_t)cmd_privacy_credent, aaa);
            }
            {
				ConParm_t aaa[]  =
                    {
                        { "Passphrase", CON_PARM_STRING, MIN_PSK_STRING_LENGTH, MAX_PSK_STRING_LENGTH, 0},
                        { "key type (hex | text) [text]", CON_PARM_OPTIONAL | CON_PARM_STRING, 0, 5, 0},
                        CON_LAST_PARM
                    };
                consoleAddToken(h, "PSKPassphrase",  "Set PSK Passphrase", (FuncToken_t)cmd_privacy_PSKPassphrase, aaa );
            }
#ifdef _WINDOWS  // TRS:HLC certificate hash for Windows
#else
            {
                ConParm_t aaa[]  = { { "Certificate Name:", CON_PARM_STRING, 0, MAX_CERT_FILE_NAME_LENGTH, 0 },
                { "Validate (yes - 1 /no - 0):", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h, "cerTificate",  "Set Certificate",(FuncToken_t)cmd_privacy_certificate, aaa);

            }
#endif
            {
                ConParm_t aaa[]  = { { "option", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
                consoleAddToken(h, "wpa_Options",  "Set WPA options", (FuncToken_t)cmd_privacy_wpa_options, aaa );
            }
            CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Wep",  "Wep" ) );
                {
                    ConParm_t aaa[]  =
                    {
                        { "Key Value", CON_PARM_STRING, 0, 64, 0},
                        { "Tx Key Index", 0, 0, 0, 0 },
                        { "Default Key (yes - 1 /no - 0)", 0, 0, 0, 0 },
                        { "key type (hex | text) [hex]", CON_PARM_OPTIONAL | CON_PARM_STRING, 0, 5, 0},
                        CON_LAST_PARM
                    };
                    consoleAddToken(h1, "Add",  "Add WEP", (FuncToken_t)cmd_privacy_addkey, aaa );
                }
                {
                    ConParm_t aaa[]  = { { "Key Index", 0, 0, 0, 0 }, CON_LAST_PARM };
                    consoleAddToken(h1, "Remove",  "Remove WEP", (FuncToken_t)cmd_privacy_removekey, aaa);
                }
                consoleAddToken(h1, "Get Default Key ID",  "Get Default Key ID", (FuncToken_t)cmd_privacy_getdefaultkey, NULL);


#ifdef EXC_MODULE_INCLUDED
            add_EXC_menu( h );
#endif/*EXC_MODULE_INCLUDED*/

    /* -------------------------------------------- Scan -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "scAn",  "Scan Service Configuration" ) );
        consoleAddToken(h, "Start",  "Start Application Scan", (FuncToken_t) cmd_Scan_Start, NULL );
        consoleAddToken(h, "sTop",  "Stop Application Scan", (FuncToken_t) cmd_Scan_Stop, NULL );
        CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "configApp",  "Configure Application Scan Params" ) );
        {
            ConParm_t aaa[]  = {
                    { "SSID", CON_PARM_STRING, 0, 33, 0 },
#ifdef TI_DBG /* limitn application scan to normal only in release version */
                    { "Scan Type", CON_PARM_RANGE, SCAN_TYPE_NORMAL_PASSIVE, SCAN_TYPE_TRIGGERED_ACTIVE, 0 },
#else
                    { "Scan Type", CON_PARM_RANGE, SCAN_TYPE_NORMAL_PASSIVE, SCAN_TYPE_NORMAL_ACTIVE, 0 },
#endif
                    { "Band", CON_PARM_RANGE, 0, 1, 0 },
                    { "Probe Request Number", CON_PARM_RANGE, 0, 255, 0 },
                    { "Probe Request Rate", CON_PARM_RANGE, 0, DRV_RATE_MASK_54_OFDM, 0 },

#ifdef TI_DBG
                    { "Tid", CON_PARM_RANGE, 0, 255, 0 },
#endif
                    { "Number of Channels", CON_PARM_RANGE, 0, 16, 0 },
                    CON_LAST_PARM };
            consoleAddToken(h1, "Global",  "Config Global Params", (FuncToken_t) cmd_Scan_app_global_config, aaa );
        }
        {
            ConParm_t aaa[]  = {
                    { "Index", CON_PARM_RANGE, 0, 30, 0 },
                    { "BSSID (xx:xx:xx:xx:xx:xx)", CON_PARM_STRING, 0, 18, 0 },
                    { "Max Dwell Time", CON_PARM_RANGE, 0, 100000000, 0 },
                    { "Min Dwell Time", CON_PARM_RANGE, 0, 100000000, 0 },
                    { "ET Condition", CON_PARM_RANGE, SCAN_ET_COND_DISABLE, SCAN_ET_COND_ANY_FRAME, 0 },
                    { "ET Frame Number", CON_PARM_RANGE, 0, 255, 0 },
                    { "TX power level", CON_PARM_RANGE, 0, MAX_TX_POWER, 0 },
                    { "Channel Number", CON_PARM_RANGE, 0, 255, 0 },
                    CON_LAST_PARM };
            consoleAddToken(h1, "Channel",  "Config Channel Params", (FuncToken_t) cmd_Scan_app_channel_config, aaa );
        }
        consoleAddToken(h1, "cLear",  "Clear All Params", (FuncToken_t) cmd_Scan_app_clear, NULL );
        consoleAddToken(h1, "Display",  "Display Params", (FuncToken_t) cmd_Scan_app_display, NULL );

        CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "configPolicy",  "Configure scan manager scan policy" ) );
        {
            ConParm_t aaa[]  = {
                    { "Normal scan interval (msec)", CON_PARM_RANGE, 0, 3600000, 5000 },
                    { "Deteriorating scan interval", CON_PARM_RANGE, 0, 3600000, 3000 },
                    { "Max Track Failures", CON_PARM_RANGE, 0, 20, 3 },
                    { "BSS list size", CON_PARM_RANGE, 0, 16, 8 },
                    { "BSS Number to start discovery", CON_PARM_RANGE, 0, 16, 4 },
                    { "Number of bands", CON_PARM_RANGE, 0, 2, 1 },
                     CON_LAST_PARM };
            consoleAddToken(h1, "Gloabal",  "Set Global policy Params", (FuncToken_t) cmd_Scan_policy_global_config, aaa );
        }

            CHK_NULL(h2 = (handle_t) consoleAddDirExt( (handle_t) h1, "Band",  "Configure band scan policy" ) );
            {
                ConParm_t aaa[]  = {
                        { "Index", CON_PARM_RANGE, 0, 1, 0 },
                        { "Band", CON_PARM_RANGE, 0, 1, 0 },
                        { "RSSI threshold", CON_PARM_RANGE| CON_PARM_SIGN, -100, 0, 0 },
                        { "Channel number for discovery cycle", CON_PARM_RANGE, 0, 30, 5 },
                        { "Number of Channels", CON_PARM_RANGE, 0, 30, 0 },
                         CON_LAST_PARM };
                consoleAddToken(h2, "Misc",   "Set misc band params",  (FuncToken_t) cmd_Scan_band_global_config, aaa );
            }
            {
                ConParm_t aaa[]  = {
                        { "Band Index", CON_PARM_RANGE, 0, 1, 0 },
                        { "Channel Index", CON_PARM_RANGE, 0, 29, 0 },
                        { "Channel", CON_PARM_RANGE, 0, 160, 0 },
                         CON_LAST_PARM };
                consoleAddToken(h2, "Channel",   "Set Channel params",  (FuncToken_t) cmd_Scan_band_channel_config, aaa );
            }
            {
                ConParm_t aaa[]  = {
                        { "Band Index", CON_PARM_RANGE, 0, 1, 0 },
                        { "Scan Type", CON_PARM_RANGE, 0, 5, 0 },
                        { "ET event", CON_PARM_RANGE, SCAN_ET_COND_DISABLE, SCAN_ET_COND_ANY_FRAME, SCAN_ET_COND_DISABLE },
                        { "ET num of frames", CON_PARM_RANGE, 0, 255,0 },
                        { "Triggering AC", CON_PARM_RANGE, 0, 255, 0 },
                        { "Scan Duration (SPS)", CON_PARM_RANGE, 0, 100000000, 2000 },
                        { "Max dwell time", CON_PARM_RANGE, 0, 100000000, 60000 },
                        { "Min dwell time", CON_PARM_RANGE, 0, 100000000, 30000 },
                        { "Probe req. number", CON_PARM_RANGE, 0, 255, 2 },

                        { "Probe req. rate", CON_PARM_RANGE, 0, DRV_RATE_MASK_54_OFDM, 0 },

                        { "TX power level", CON_PARM_RANGE, 0, MAX_TX_POWER, 0 },
                         CON_LAST_PARM };
                consoleAddToken(h2, "Track",   "Set tracking method params",  (FuncToken_t) cmd_Scan_band_track_config, aaa );
            }
            {
                ConParm_t aaa[]  = {
                        { "Band Index", CON_PARM_RANGE, 0, 1, 0 },
                        { "Scan Type", CON_PARM_RANGE, 0, 5, 0 },
                        { "ET event", CON_PARM_RANGE, SCAN_ET_COND_DISABLE, SCAN_ET_COND_ANY_FRAME, SCAN_ET_COND_DISABLE },
                        { "ET num of frames", CON_PARM_RANGE, 0, 255,0 },
                        { "Triggering AC", CON_PARM_RANGE, 0, 255, 0 },
                        { "Scan Duration (SPS)", CON_PARM_RANGE, 0, 100000000, 2000 },
                        { "Max dwell time", CON_PARM_RANGE, 0, 100000000, 60000 },
                        { "Min dwell time", CON_PARM_RANGE, 0, 100000000, 30000 },
                        { "Probe req. number", CON_PARM_RANGE, 0, 255, 2 },

                        { "Probe req. rate", CON_PARM_RANGE, 0, DRV_RATE_MASK_54_OFDM, 0 },

                        { "TX power level", CON_PARM_RANGE, 0, MAX_TX_POWER, 0 },
                         CON_LAST_PARM };
                consoleAddToken(h2, "Discovery",   "Set Discovery method params",  (FuncToken_t) cmd_Scan_band_discover_config, aaa );
            }
            {
                ConParm_t aaa[]  = {
                        { "Band Index", CON_PARM_RANGE, 0, 1, 0 },
                        { "Scan Type", CON_PARM_RANGE, 0, 5, 0 },
                        { "ET event", CON_PARM_RANGE, SCAN_ET_COND_DISABLE, SCAN_ET_COND_ANY_FRAME, SCAN_ET_COND_DISABLE },
                        { "ET num of frames", CON_PARM_RANGE, 0, 255,0 },
                        { "Triggering AC", CON_PARM_RANGE, 0, 255, 0 },
                        { "Scan Duration (SPS)", CON_PARM_RANGE, 0, 100000000, 2000 },
                        { "Max dwell time", CON_PARM_RANGE, 0, 100000000, 60000 },
                        { "Min dwell time", CON_PARM_RANGE, 0, 100000000, 30000 },
                        { "Probe req. number", CON_PARM_RANGE, 0, 255, 2 },

                        { "Probe req. rate", CON_PARM_RANGE, 0, DRV_RATE_MASK_54_OFDM, 0 },

                        { "TX power level", CON_PARM_RANGE, 0, MAX_TX_POWER, 0 },
                         CON_LAST_PARM };
                consoleAddToken(h2, "Immediate",   "Set Immediate method params",  (FuncToken_t) cmd_Scan_band_immed_config, aaa );
            }


        consoleAddToken(h1, "Display",  "Display Policy Params", (FuncToken_t) cmd_Scan_policy_display, NULL );
        consoleAddToken(h1, "Clear",  "Clear Polciy Params", (FuncToken_t) cmd_Scan_policy_clear, NULL );
        consoleAddToken(h1, "Store",  "Send policy to scan manager", (FuncToken_t) cmd_Scan_policy_store, NULL );
        consoleAddToken(h1, "bsslisT", "Display BSS list", (FuncToken_t) cmd_Scan_get_bss_list, NULL );


	/************ ROAMING manager commands - start  ********************/
    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "roaminG",  "Roaming Manager configuration" ) );
	consoleAddToken(h, "Enable",  "Enable Internal Roaming", (FuncToken_t) cmd_Roaming_enable, NULL );
	consoleAddToken(h, "Disable",  "Disable Internal Roaming", (FuncToken_t) cmd_Roaming_disable, NULL );
	{
		ConParm_t aaa[]  = {
				{ "Low pass filter time", CON_PARM_DEFVAL, 0, 1440, 30 }, CON_LAST_PARM };
		consoleAddToken(h, "Low pass filter",  "Time in sec ", (FuncToken_t) cmd_Roaming_lowPassFilter, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "Quality threshold", CON_PARM_DEFVAL | CON_PARM_SIGN, -150, 0, -100 }, CON_LAST_PARM };
		consoleAddToken(h, "Quality threshold",  "Quality indicator", (FuncToken_t) cmd_Roaming_qualityIndicator, aaa );
	}

	consoleAddToken(h, "Get ",  "Get Roaming config params ", (FuncToken_t) cmd_Roaming_getConfParams, NULL );

    CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Thresholds",  "Set Roaming MNGR triggers thresholds" ) );
	{
		ConParm_t aaa[]  = {
				{ "Tx retry", CON_PARM_DEFVAL, 0, 255, 20 }, CON_LAST_PARM };
		consoleAddToken(h1, "Tx retry ",  "Consecutive number of TX retries", (FuncToken_t) cmd_Roaming_dataRetryThreshold, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "Bss loss", CON_PARM_DEFVAL, 1, 255, 4 }, CON_LAST_PARM };
		consoleAddToken(h1, "Bss loss ",  "Number of TBTTs", (FuncToken_t) cmd_Roaming_numExpectedTbttForBSSLoss, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "tx Rate threshold", CON_PARM_DEFVAL, 0, 54, 2 }, CON_LAST_PARM };
				consoleAddToken(h1, "tx Rate threshold ",  "TX rate (fallback) threshold", (FuncToken_t) cmd_Roaming_txRateThreshold, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "Low rssi threshold", CON_PARM_DEFVAL | CON_PARM_SIGN, -150, 0, -80 }, CON_LAST_PARM };

				consoleAddToken(h1, "Low rssi threshold ",  "Low RSSI threshold", (FuncToken_t) cmd_Roaming_lowRssiThreshold, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "low Snr threshold", CON_PARM_DEFVAL, 0, 255, 10 }, CON_LAST_PARM };
		consoleAddToken(h1, "low Snr threshold ",  "Low SNR threshold", (FuncToken_t) cmd_Roaming_lowSnrThreshold, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "low Quality for scan", CON_PARM_DEFVAL | CON_PARM_SIGN, -150, -40, -85 }, CON_LAST_PARM };
				consoleAddToken(h1, "low Quality for scan ",  "Increase the background scan", (FuncToken_t) cmd_Roaming_lowQualityForBackgroungScanCondition, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "Normal quality for scan", CON_PARM_DEFVAL | CON_PARM_SIGN, -150, -40, -70 }, CON_LAST_PARM };
				consoleAddToken(h1, "Normal Quality for scan ",  "Reduce the background scan", (FuncToken_t) cmd_Roaming_normalQualityForBackgroungScanCondition, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "Rssi filter weight", CON_PARM_DEFVAL, 0, 100, 10 }, CON_LAST_PARM };
				consoleAddToken(h1, "rssI filter weight ",  "Set weight for the last RSSI value in the AVG calculation", (FuncToken_t) cmd_Roaming_rssiFilterWeight, aaa );
	}
	{
		ConParm_t aaa[]  = {
				{ "Snr filter weight", CON_PARM_DEFVAL, 0, 100, 10 }, CON_LAST_PARM };
				consoleAddToken(h1, "snr Filter weight ",  "Set weight for the last SNR value in the AVG calculation", (FuncToken_t) cmd_Roaming_snrFilterWeight, aaa );
	}

	/************ ROAMING manager commands - end  ********************/

    /* -------------------------------------------- QOS -------------------------------------------- */


    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "qOs",  "Quality of service" ) );

		CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Upsd",  "UPSD Sub-menu" ) );
        {       ConParm_t TspecParams[]  = {
                { "UserPriority", CON_PARM_RANGE, 0, 7, 1  },
                { "NominalMSDUsize", CON_PARM_RANGE, 1, 2312, 1  },
                { "MeanDataRate (Bps units)", CON_PARM_RANGE, 0, 54000000, 0 },
                { "MinimumPHYRate (Mbps units)", CON_PARM_RANGE , 0, 54, 0  },
                { "SurplusBandwidthAllowance", CON_PARM_RANGE , 0, 7, 0 },
                { "UPSD Mode (0 - Legacy, 1 - U-APSD)", CON_PARM_RANGE , 0, 1, 0 },
                    CON_LAST_PARM };
                consoleAddToken(h1, "Add",  "Add TSPEC", (FuncToken_t) cmd_add_tspec, TspecParams );
        }
        {
            ConParm_t UPid[]  = { { "User priority", CON_PARM_RANGE, 0, 7, 1  }, CON_LAST_PARM };
            consoleAddToken(h1, "Get",  "Get TSPEC Params", (FuncToken_t) cmd_get_tspec_params, UPid );
        }
        {
            ConParm_t UPid[]  = { { "UserPriority", CON_PARM_RANGE, 0, 7, 1  },
                                  { "ReasonCode", CON_PARM_RANGE, 32, 45, 32  }, CON_LAST_PARM };
            consoleAddToken(h1, "Delete",  "Delete TSPEC", (FuncToken_t) cmd_delete_tspec, UPid );
        }

        consoleAddToken(h1, "aP params", "Get AP QoS parameters", (FuncToken_t) cmd_get_ap_qos_params, NULL );
        consoleAddToken(h1, "ap Capabilities", "Get AP QoS capabilities parameters", (FuncToken_t) cmd_get_ap_qos_capabilities, NULL );

        {
            ConParm_t ACid[]  = { { "AC", CON_PARM_RANGE, 0, 3, 3  }, CON_LAST_PARM };
            consoleAddToken(h1, "ac Status", "Get Current AC Status", (FuncToken_t) cmd_get_ac_status, ACid );
        }

            {
                ConParm_t MediumUsageParams[]  = {
                    { "AC", CON_PARM_RANGE, 0, 3, 3  },
                    { "HighThreshold", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 100, 1  },
                    { "LowThreshold", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 100, 1 },
                     CON_LAST_PARM };
                    consoleAddToken(h1, "Medium usage", "Medium usage threshold", (FuncToken_t) cmd_medium_usage_threshold, MediumUsageParams );
            }
            {
                ConParm_t PhyRateParams[]  = {
                    { "AC", CON_PARM_RANGE, 0, 3, 3  },
                    { "HighThreshold", CON_PARM_RANGE | CON_PARM_OPTIONAL, 1, 54, 1  },
                    { "LowThreshold", CON_PARM_RANGE | CON_PARM_OPTIONAL, 1, 54, 1 },
                     CON_LAST_PARM };
                    consoleAddToken(h1, "phy Rate", "PHY rate threshold", (FuncToken_t) cmd_phy_rate_threshold, PhyRateParams );
            }
        consoleAddToken(h1, "dEsired ps mode", "Get desired PS mode", (FuncToken_t) cmd_get_desired_ps_mode, NULL );

    CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Classifier",  "Classifier sub-menu" ) );
        {
            ConParm_t aaa[]  = {{ "con0 Port = ", CON_PARM_RANGE, 0, 65535, 0  },
                                { "con0 Pri = ", CON_PARM_RANGE, 0, 7, 0  },
                                { "con1 Port = ", CON_PARM_RANGE, 0, 65535, 0  },
                                { "con1 Pri = ", CON_PARM_RANGE, 0, 7, 0  },
                                { "con2 Port = ", CON_PARM_RANGE, 0, 65535, 0  },
                                { "con2 Pri = ", CON_PARM_RANGE, 0, 7, 0  },
                                { "con3 Port = ", CON_PARM_RANGE, 0, 65535, 0  },
                                { "con3 Pri = ", CON_PARM_RANGE, 0, 7, 0  },
                                { "cons Ip1 = ", CON_PARM_RANGE, 0,255 , 0   },
                                { "cons Ip2 = ", CON_PARM_RANGE, 0,255 , 0   },
                                { "cons Ip3 = ", CON_PARM_RANGE, 0,255 , 0   },
                                { "cons Ip4 = ", CON_PARM_RANGE, 0,255 , 0   },
                                CON_LAST_PARM };
            consoleAddToken(h1, "TxClassifier", "Config Tx Classifier", (FuncToken_t) cmd_config_tx_classifier, aaa );
        }

        {       ConParm_t aaa[]  = {
                { "Type", CON_PARM_RANGE, DSCP_CLSFR, CLSFR_TYPE_MAX, 0  },
                { "D-Tag", CON_PARM_RANGE, CLASSIFIER_DTAG_MIN, CLASSIFIER_DTAG_MAX, CLASSIFIER_DTAG_DEF  },
                { "Param1", CON_PARM_RANGE, 0, 65535, 0 },
                { "Ip1", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0  },
                { "Ip2", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
                { "Ip3", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
				{ "Ip4", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
                    CON_LAST_PARM };
                consoleAddToken(h1, "Insert ",  "Insert new classification entry", (FuncToken_t) cmd_insert_clsfr_entry, aaa );
        }

        {       ConParm_t aaa[]  = {
                { "Type", CON_PARM_RANGE, DSCP_CLSFR, CLSFR_TYPE_MAX, 0  },
                { "D-Tag", CON_PARM_RANGE, CLASSIFIER_DTAG_MIN, CLASSIFIER_DTAG_MAX, CLASSIFIER_DTAG_DEF  },
                { "Param1", CON_PARM_RANGE, 0, 65535, 0 },
                { "Ip1", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0  },
                { "Ip2", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
                { "Ip3", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
				{ "Ip4", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
                    CON_LAST_PARM };
                consoleAddToken(h1, "Remove",  "Remove classification entry", (FuncToken_t) cmd_remove_clsfr_entry, aaa );
        }

       {
            ConParm_t aaa[]  = {
            { "acID", CON_PARM_RANGE, 0, 3, 0  },
            { "MaxLifeTime", CON_PARM_RANGE , 0, 1024, 0  },
            { "Reserved and ignored (ShortMaxRetries)", CON_PARM_RANGE , 0, 255, 0 },
            { "Reserved and ignored (LongMaxRetries)", CON_PARM_RANGE , 0, 255, 0 },
            { "Reserved and ignored (RxTimeout)", CON_PARM_RANGE , 0, 65535, 0  },
            { "Voice delivery Protocol (VO QUEUE ONLY : 0 - None, 1 - PS POLL)", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 1, 0 },
			{ "PS Delivery Protocol (1 - U-APSD, 2 - Legacy)", CON_PARM_RANGE | CON_PARM_OPTIONAL, 1 /*PS_SCHEME_UPSD_TRIGGER*/, 2/*PS_SCHEME_LEGACY_PSPOLL*/, 2/*PS_SCHEME_LEGACY_PSPOLL*/},
                CON_LAST_PARM };
            consoleAddToken(h, "QosParams ",  "Set QOS Parameters", (FuncToken_t) cmd_set_qos_params, aaa );
        }

        {
            ConParm_t aaa[]  = {
            { "dtag0_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag1_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag2_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag3_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag4_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag5_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag6_to_ac", CON_PARM_RANGE, 0, 3, 0  },
            { "dtag7_to_ac", CON_PARM_RANGE, 0, 3, 0  },
                CON_LAST_PARM };
            consoleAddToken(h, "Set_dtag2ac_mapping_table ",  "Set dtag2ac mapping table", (FuncToken_t) cmd_set_dtag_to_ac_mapping_table, aaa );
        }

        {
            ConParm_t aaa[]  = {
            { "Enable", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 1, 0  },
            { "vadTimerDuration", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 100, 0  },
                CON_LAST_PARM };
            consoleAddToken(h, "setVad ",  "Set VAD", (FuncToken_t) cmd_set_vad, aaa );
        }
        {
            ConParm_t aaa[]  = { { "AC", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 3, 3  }, CON_LAST_PARM };
            consoleAddToken(h, "Poll AP packets", "Poll Ap Packets", (FuncToken_t) cmd_poll_ap_packets, aaa );
        }

		{
            ConParm_t aaa[]  = {
            { "PsPoll", CON_PARM_RANGE, 0, 65000, 0  },
            { "UPSD", CON_PARM_RANGE , 0, 65000, 0  },
                CON_LAST_PARM };
            consoleAddToken(h, "Rx TimeOut ",  "Rx TimeOut ", (FuncToken_t) cmd_set_rxTimeOut_params, aaa );
        }
		{
            ConParm_t aaa[]  = {
            { "MaxRxLifeTime", CON_PARM_OPTIONAL|CON_PARM_RANGE, 0, 0xffffffff, 0  },
                CON_LAST_PARM };
            consoleAddToken(h, "Max Rx Lifetime",  "Max Rx Lifetime", (FuncToken_t) cmd_MaxRxLifetime_params, aaa );
        }





        /* -------------------------------------------- Power Management -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "poWer",  "Power Management" ) );
        {
            	/* Set Power Mode Command */
            	ConParm_t powerModeCmd[]  = {
                    { "PowerMode", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 3, 1 }, /* Min/Max/Def */
                    CON_LAST_PARM };
            	consoleAddToken(h, "set_Power_mode",  "Set user power mode", (FuncToken_t) cmd_set_power_mode, powerModeCmd );

        }
        {
		/* Set Power Save Power level Command */
            	ConParm_t powerSavePowerLevelCmd[]  = {
                    { "PowerSavePowerLevel", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 2, 2 }, /* Min/Max/Def */
                    CON_LAST_PARM };
            consoleAddToken(h, "set_powersave_powerLevel",  "Set the Power level during PowerSave", (FuncToken_t) cmd_set_PowerSave_PowerLevel, powerSavePowerLevelCmd );

        }
		{
		/* Set default Power level Command */
            	ConParm_t defaultPowerLevelCmd[]  = {
                    { "DefaultPowerLevel", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 2, 2 }, /* Min/Max/Def */
                    CON_LAST_PARM };
            consoleAddToken(h, "set_deFault_powerlevel",  "Set the default power level", (FuncToken_t) cmd_set_Default_PowerLevel, defaultPowerLevelCmd );

        }
		{
		/* Set doze mode in auto power mode */
            	ConParm_t powerSaveDozeMode[]  = {
                    { "DozeModeInAuto", CON_PARM_RANGE | CON_PARM_OPTIONAL, AUTO_POWER_MODE_DOZE_MODE_MIN_VALUE, AUTO_POWER_MODE_DOZE_MODE_MAX_VALUE, AUTO_POWER_MODE_DOZE_MODE_DEF_VALUE },
                    CON_LAST_PARM };
            consoleAddToken(h, "set_doZe_mode_in_auto",  "Set doze mode in auto power mode", (FuncToken_t) cmd_set_DozeModeInAutoPowerLevel, powerSaveDozeMode );

        }
        {
            ConParm_t TrafficIntensityParams[]  = {
                { "HighThreshold (packets/sec)", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 1000, 100  },
                { "LowThreshold (packets/sec)", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 1000, 25 },
                { "CheckInterval (ms)", CON_PARM_RANGE | CON_PARM_OPTIONAL, 100, 10000, 1000 },
                 CON_LAST_PARM };
                consoleAddToken(h, "traffic_Thresholds", "Set/Get traffic intensity thresholds", (FuncToken_t) cmd_traffic_intensity_threshold, TrafficIntensityParams );
        }
         consoleAddToken(h, "eNable",  "enable traffic intensity events", (FuncToken_t) cmd_enable_traffic_events, NULL );
         consoleAddToken(h, "Disable",  "disable traffic intensity events", (FuncToken_t) cmd_disable_traffic_events, NULL );

       /* -------------------------------------------- Events -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "eVents",  "Events" ) );
        {
            ConParm_t aaa[]  = { { "type", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Register",  "IPC events", (FuncToken_t)cmd_events_register, aaa);
        }
        {
            ConParm_t aaa[]  = { { "type", CON_PARM_OPTIONAL, 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Unregister",  "IPC events", (FuncToken_t)cmd_events_unregister, aaa);
        }
    CHK_NULL(h = (handle_t) consoleAddDirExt( NULL, "File",  "restore configuration" ) );
        {
            ConParm_t aaa[]  = { { "filename", CON_PARM_STRING, 0, 128, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Load",  "Load profile", (FuncToken_t) cmd_file_load, aaa );
        }

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "Bt CoExsistance",  "BT - Wlan CoExsistance" ) );
        {
			ConParm_t aaa[]  = { { "enable", CON_PARM_RANGE | CON_PARM_OPTIONAL,
								SOFT_GEMINI_ENABLED_MIN, SOFT_GEMINI_ENABLED_MAX, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Enable", "Enable BT Coexistense", (FuncToken_t) cmd_bt_coe_enable, aaa );
        }
        {
			ConParm_t aaa[]  = { { "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								{ "rate", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 255, 0 },
								CON_LAST_PARM };
            consoleAddToken(h, "Rate", "Select Rates", (FuncToken_t) cmd_bt_coe_rate, aaa );
        }
		{
            ConParm_t aaa[]  = {
							{ "wlanRxMinRateToRespectBtHp", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_MIN , SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_MAX, 0  },
							{ "btHpMaxTime", CON_PARM_RANGE | CON_PARM_OPTIONAL,
							SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_MIN, SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_MAX, 0  },
							{ "wlanHpMaxTime", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_MIN, SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_MAX, 0  },
							{ "senseDisableTimer", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_MIN, SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_MAX, 0  },
							{ "protectiveRxTimeBeforeBtHp", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_MAX, 0  },
							{ "protectiveTxTimeBeforeBtHp", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_MAX, 0  },
							{ "protectiveRxTimeBeforeBtHpFastAp", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_MAX, 0  },
							{ "protectiveTxTimeBeforeBtHpFastAp", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_MAX, 0  },
							{ "protectiveWlanCycleTimeForFastAp", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_MIN, SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_MAX, 0  },
                            { "btAntiStarvationPeriod", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_MIN , SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_MAX, 0  },
							{ "timeoutNextBtLpPacket", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_MIN, SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_MAX,0  },
							{ "wakeUpTimeBeforeBeacon", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_MIN , SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_MAX, 0  },
							{ "hpdmMaxGuardTime", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_MIN , SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_MAX, 0  },
							{ "timeoutNextWlanPacket", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_MIN , SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_MAX, 0  },
							{ "sgAntennaType", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_MIN, SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_MAX , 0  },
							{ "signalingType", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_SIGNALING_TYPE_MIN, SOFT_GEMINI_PARAMS_SIGNALING_TYPE_MAX , 0  },
							{ "afhLeverageOn", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_MIN, SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_MAX, 0  },
							{ "numberQuietCycle", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_MIN, SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_MAX , 0  },
							{ "maxNumCts", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_MAX_NUM_CTS_MIN, SOFT_GEMINI_PARAMS_MAX_NUM_CTS_MAX, 0  },
							{ "numberOfWlanPackets", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_MIN, SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_MAX,0  },
							{ "numberOfBtPackets", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_MIN, SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_MAX,0  },
							{ "numberOfMissedRxForAvalancheTrigger", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_MIN, SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_MAX, 0  },
							{ "wlanElpHpSupport", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_ELP_HP_MIN, SOFT_GEMINI_PARAMS_ELP_HP_MAX, 0  },
							{ "btAntiStarvationNumberOfCyclesWithinThePeriod", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_MIN , SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_MAX, 0  },
                            { "ackModeDuringBtLpInDualAnt", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
                            SOFT_GEMINI_PARAMS_ACK_MODE_MIN , SOFT_GEMINI_PARAMS_ACK_MODE_MAX, 0  },
                            { "allowPaSdToggleDuringBtActivityEnable", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_ALLOW_PA_SD_MIN , SOFT_GEMINI_PARAMS_ALLOW_PA_SD_MAX, 0  },
							{ "sgAutoModeNoCts", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_MIN , SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_MAX, 0  },
							{ "numOfBtHpRespectedReq", CON_PARM_RANGE | CON_PARM_OPTIONAL ,
							SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_MIN , SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_MAX, 0  },
							CON_LAST_PARM };

			consoleAddToken(h, "Config",  "Parameters configuration", (FuncToken_t) cmd_bt_coe_config, aaa );
		}
        {
			ConParm_t aaa[]  = { { "status", CON_PARM_RANGE | CON_PARM_OPTIONAL, 0, 3, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Status", "Get status", (FuncToken_t) cmd_bt_coe_get_status, aaa );
        }
#ifdef EXC_MODULE_INCLUDED
		/************ MEASUREMENT commands - start  ********************/
		CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "measuremenT",  "Measurement configuration" ) );
		consoleAddToken(h, "Enable",  "Enable Measurement", (FuncToken_t) cmd_Measurement_enable, NULL );
		consoleAddToken(h, "Disable",  "Disable Measurement", (FuncToken_t) cmd_Measurement_disable, NULL );
		{
			ConParm_t aaa[]  = {
					{ "Change max duration", CON_PARM_RANGE, 0, 2000, 300 }, CON_LAST_PARM };
			consoleAddToken(h, "Max duration",  "Time in msec ", (FuncToken_t) cmd_Measurement_setMaxDuration, aaa );
		}
		/************ MEASUREMENT commands - end  ********************/
#endif /* EXC_MODULE_INCLUDED*/

#ifdef TI_DBG

      /* -------------------------------------------- Report -------------------------------------------- */

    CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) NULL, "Report",  "Debug features" ) );
        {
            ConParm_t aaa[]  =
			{
				{ "module table", CON_PARM_STRING | CON_PARM_OPTIONAL , WLAN_MAX_LOG_MODULES, WLAN_MAX_LOG_MODULES, 0 },
				CON_LAST_PARM };
            consoleAddToken(h1, "Set",  "set report module table", (FuncToken_t) cmd_report_set, aaa );
        }
        {
            ConParm_t aaa[]  =
			{
				{ "module", CON_PARM_OPTIONAL, 0, 0, 0 },
				CON_LAST_PARM
			};
            consoleAddToken(h1, "Add",  "set report for specified module", (FuncToken_t) cmd_report_add, aaa );
        }
        {
			ConParm_t aaa[]  =
			{
				{ "module", CON_PARM_OPTIONAL, 0, 0, 0 },
				CON_LAST_PARM
			};
            consoleAddToken(h1, "Clear",  "clear report for specified module", (FuncToken_t) cmd_report_clear, aaa );
        }
        {
            ConParm_t aaa[]  = { { "level", CON_PARM_OPTIONAL , 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h1, "Level",  "set report severity level", (FuncToken_t) cmd_report_severity_level, aaa );
        }
        {
            ConParm_t aaa[]  = { { "osDbgState", CON_PARM_OPTIONAL , 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h1, "Osdbgstate",  "set OS abstraction layer debug dtate", (FuncToken_t) cmd_report_os_dbg_state, aaa );
        }

      /* -------------------------------------------- Debug -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "dEbug",  "Debug features" ) );
        {
            ConParm_t aaa[]  = {{ "reg_num", 0, 0, 0, 0 },
                                { "value", CON_PARM_OPTIONAL , 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "Register",  "read/write HW register", (FuncToken_t) cmd_hw_register, aaa );
        }

        {
            ConParm_t aaa[]  = {{ "func_num", CON_PARM_OPTIONAL, 0, 0, 0 },
                                { "param", CON_PARM_OPTIONAL , 0, 0, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "print",  "print driver debug info", (FuncToken_t) cmd_debug_driver_print, aaa );
        }


		{
			ConParm_t aaa[]  = { { "func_num", CON_PARM_OPTIONAL, 0, 0, 0 },
											{ "string param", CON_PARM_LINE, 0, 128, 0 },
											CON_LAST_PARM };
			consoleAddToken(h, "Buffer",  "Pass the buffer to the driver", (FuncToken_t)cmd_debug_buffer_put, aaa );
		}

#ifdef DRIVER_PROFILING
        consoleAddToken(h, "proFile report",  "Show driver resource usage", (FuncToken_t) cmd_profile_report, NULL );
        {
            ConParm_t aaa[] = {{ "command_type", CON_PARM_DEFVAL | CON_PARM_RANGE, 1, 3, 3 },
                                { "resolution", CON_PARM_OPTIONAL , 0, 0, 0 },
                                CON_LAST_PARM
                                };
            consoleAddToken(h, "Cpu estimator command",  "start/stop/reset cpu estimator", (FuncToken_t) cmd_profile_cpu_estimator_command, aaa );
        }

#endif


#endif /*TI_DBG*/

      /* -------------------------------------------- PLT -------------------------------------------- */

    CHK_NULL(h = (handle_t) consoleAddDirExt( (handle_t) NULL, "pLt",  "PLatform table Testings" ) );
        CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Register",  "Register Read/Write" ) );
        {
            ConParm_t aaa[]  = {{ "address(hex)", CON_PARM_STRING, 1, 8, 0 },
                                 CON_LAST_PARM };
            consoleAddToken(h1, "Read",  "read register", (FuncToken_t) cmd_PLT_RegisterRead, aaa );
        }

        {
            ConParm_t aaa[]  = {{ "address(hex)", CON_PARM_STRING, 1, 8, 0 },
                                { "value(hex)", CON_PARM_STRING, 1, 8, 0 },
                                 CON_LAST_PARM };
            consoleAddToken(h1, "Write",  "write register", (FuncToken_t) cmd_PLT_RegisterWrite, aaa );
        }

		{
            ConParm_t aaa[]  = {{ "Band", CON_PARM_OPTIONAL|CON_PARM_RANGE, 0, 2, 0 },
            { "Channel", CON_PARM_OPTIONAL|CON_PARM_RANGE , 1, 161, 0 }, CON_LAST_PARM };
            consoleAddToken(h, "rAdio tune",  "Set the RX channel", (FuncToken_t) cmd_PLT_RadioTune, aaa );
        }

		CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "rx Per",  "RX PER test" ) );
			{
				ConParm_t aaa[] = {CON_LAST_PARM};
				consoleAddToken(h1, "Start",  "Stop RX PER counters", (FuncToken_t) cmd_PLT_RxPerStart, aaa );
				consoleAddToken(h1, "stoP",  "Stop RX PER counters", (FuncToken_t) cmd_PLT_RxPerStop, aaa );
				consoleAddToken(h1, "Clear",  "Clear RX PER counters", (FuncToken_t) cmd_PLT_RxPerClear, aaa );
				consoleAddToken(h1, "Get",  "Get RX PER counters", (FuncToken_t) cmd_PLT_RxPerGet, aaa );
			}

		CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Tx",  "TX test test" ) );
				{
                    ConParm_t aaa[]  = {{ "Band", CON_PARM_OPTIONAL|CON_PARM_RANGE, 0, 2, 0 },
                    { "Channel", CON_PARM_OPTIONAL , 1, 161, 0 }, CON_LAST_PARM };
				consoleAddToken(h1, "Cw",  "Start CW test", (FuncToken_t) cmd_PLT_TxCW, aaa );
				}
                {
                    ConParm_t aaa[]  = {{ "Band (0 - 2.4Ghz(B/G), 1 - 5Ghz(A), 2 - Japan(4.9Ghz))", CON_PARM_OPTIONAL|CON_PARM_RANGE, 0, 2, 0 },
                    { "Channel: (1..14(2.4Ghz), 1..180(5Ghz))", CON_PARM_OPTIONAL|CON_PARM_RANGE , 1, 161, 14 },
                    { "Rate: 1-1M,2-2M,3-5.5M,4-11M,6-6M,7-9M,8-12M,9-18M.10-24M,11-36M,12-48M,13-54M", CON_PARM_OPTIONAL|CON_PARM_RANGE , 1, 13, 13 },
                    { "preamble (0-long, 1-short)", CON_PARM_OPTIONAL|CON_PARM_RANGE , 0, 1, 1 },
                    { "InterPacketDelay- Delay between packets (uSec)", CON_PARM_OPTIONAL|CON_PARM_RANGE , 0, 0xffffffff, 0 },
                    { "Number of TX frames (0 - endless)", CON_PARM_OPTIONAL|CON_PARM_RANGE , 0, 0xffffffff, 100 },
                    { "Test mode (5-Random data, 9-ZOZO(0,1,0,1,...))", CON_PARM_OPTIONAL|CON_PARM_RANGE , 5, 9, 9 },
                    { "Sequance number mode (0 - fixed, 1 - incremented)", CON_PARM_OPTIONAL|CON_PARM_RANGE , 0, 1, 0 },
                    { "packet data length [bytes] (0 - 2284)", CON_PARM_OPTIONAL|CON_PARM_RANGE , 0, 2284, 100 },
                    { "peer mac address (xx:xx:xx:xx:xx:xx)", CON_PARM_STRING, 0, 18, 0 },
									CON_LAST_PARM };
									consoleAddToken(h1, "coNtinues",  "Start TX continues test", (FuncToken_t) cmd_PLT_TxContinues, aaa );

				}
				{
				ConParm_t aaa[] = {CON_LAST_PARM};
				consoleAddToken(h1, "Stop",  "Stop TX tests", (FuncToken_t) cmd_PLT_TxStop, aaa );
				}
		CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Mib",  "Mibs" ) );
			   consoleAddToken(h1, "Counter table",  "Read_MIB_CounterTable", (FuncToken_t) cmd_PLT_MIB_CounterTable, NULL);
			   consoleAddToken(h1, "station Id",  "MIB_Station_id", (FuncToken_t) cmd_PLT_MIB_StationID, NULL );

                /* -------------------------------------------- PLT Calibrations -------------------------------------------- */
                CHK_NULL(h1 = (handle_t) consoleAddDirExt( (handle_t) h, "Calibration",  "RX/TX calibrations" ) );
                {
                    ConParm_t aaa[]  = {
                    { "Expected Rssi (Input signal generated power [1/16 db])", CON_PARM_RANGE|CON_PARM_SIGN, -2000, 2000, 0 },
                    { "Channel", CON_PARM_RANGE|CON_PARM_DEFVAL , 1, 161, 1 },
                    { "Band (0- 2.4Ghz, 1- 5Ghz, 2- 4.9Ghz)", CON_PARM_RANGE|CON_PARM_DEFVAL , 0, 2, 0 },
                    { "Interval between Samples (uSec)", CON_PARM_OPTIONAL|CON_PARM_DEFVAL , 0, 0xffff, 100 },
                    { "Number of samples", CON_PARM_OPTIONAL|CON_PARM_RANGE|CON_PARM_DEFVAL , 1, 2000, 1000 },
                    CON_LAST_PARM };
                    consoleAddToken(h1, "RX",  "RX calibration", (FuncToken_t) cmd_PLT_RxCal, aaa );
                }

                CHK_NULL(h2 = (handle_t) consoleAddDirExt( (handle_t) h1, "TX",  "TX calibrations" ) );
				{
					ConParm_t aaa[]  = {{ "Ref Tx power dBM/10 ",CON_PARM_RANGE, MIN_TX_POWER, MAX_TX_POWER, 0 },
										CON_LAST_PARM };
					consoleAddToken(h2, "Start",  "Start TX calibration", (FuncToken_t) cmd_PLT_TxCalStart, aaa );
				}
                consoleAddToken(h2, "stoP",  "Stop TX calibration", (FuncToken_t) cmd_PLT_TxCalStop, NULL );
                consoleAddToken(h2, "gain Get",  "Gain get", (FuncToken_t) cmd_PLT_TxCalGainGet, NULL );
                {
                    ConParm_t aaa[]  = {{ "Gain adjust",CON_PARM_RANGE|CON_PARM_SIGN, 0, 124, 0 },
                        CON_LAST_PARM };
                    consoleAddToken(h2, "gain Adjust",  "Set gain", (FuncToken_t) cmd_PLT_TxCalGainAdjust, aaa );
                }

                consoleAddToken(h1, "Get_NVS_Buffer",  "Get the NVS buffers needed for update", (FuncToken_t) cmd_PLT_RxTxCalNVSUpdateBuffer, NULL );

                /* -------------------------------------------- Root -------------------------------------------- */

	consoleAddToken(NULL, "aboUt",  "About", (FuncToken_t) cmd_show_about, NULL );
    consoleAddToken(NULL, "Quit",  "quit", (FuncToken_t) quit_func, NULL );
#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */

    return 0;
}


    /*  Return '0' if success */
int init_driver( char *adapter_name, char *eeprom_file_name, char *init_file_name, char *firmware_file_name )
{
#ifndef _WINDOWS
    FILE *f1=NULL, *f2=NULL, *f3 = NULL;
    UINT32 eeprom_image_length=0;
    UINT32 init_file_length=0;
    UINT32 firmware_image_length = 0;
    UINT32 req_size;
    tiwlan_dev_init_t *init_info=NULL;
#endif
    int rc = -1;
	ConParm_t param;
    tiUINT32 tmpData = 1;

//TRS:MEB add ability to convert adapter_name to tiCHAR type
#ifndef _WINDOWS
    if( !adapter_name || !*adapter_name )
        return rc;

    g_id_adapter = TI_AdapterInit( adapter_name );

#else
#endif
//TRS end

#ifdef _WINDOWS
#endif

#ifndef _WINDOWS
    /* Send init request to the driver */
    if (eeprom_file_name &&
        (f1 = fopen(eeprom_file_name, "r"))!=NULL)
    {
        if (fseek(f1, 0, SEEK_END))
        {
            fprintf(stderr, "Cannot seek eeprom image file <%s>\n", eeprom_file_name);
            goto init_driver_end;
        }
        eeprom_image_length = ftell(f1);
        printf("NVS size = %d\n", eeprom_image_length); /* Dm: */
        rewind(f1);
    }
#ifdef FIRMWARE_DYNAMIC_LOAD
    if (firmware_file_name &&
        (f2 = fopen(firmware_file_name, "r"))!=NULL)
    {
        if (fseek(f2, 0, SEEK_END))
        {
            fprintf(stderr, "Cannot seek firmware file <%s>\n", firmware_file_name);
            goto init_driver_end;
        }
        firmware_image_length = ftell(f2);
        rewind(f2);
    }
#endif
    if (init_file_name &&
        (f3 = fopen(init_file_name, "r"))!=NULL)
    {
        if (fseek(f3, 0, SEEK_END))
        {
            fprintf(stderr, "Cannot seek init file <%s>\n", init_file_name);
            goto init_driver_end;
        }
        init_file_length = ftell(f3);
        rewind(f3);
    }

    /* Now when we can calculate the request length. allocate it and read the files */
    req_size = offsetof(tiwlan_dev_init_t, data)+ eeprom_image_length + (init_file_length+1) + firmware_image_length;
    init_info = (tiwlan_dev_init_t *)malloc(req_size);
    if (!init_info)
    {
        fprintf(stderr, "No memory to allocate init request (%d bytes)\n", req_size);
        goto init_driver_end;
    }
    init_info->eeprom_image_length   = eeprom_image_length;
    init_info->firmware_image_length = firmware_image_length;
    init_info->init_file_length      = init_file_length;
    if (eeprom_image_length &&
        fread(&init_info->data[0], 1, eeprom_image_length, f1)<eeprom_image_length)
    {
        fprintf(stderr, "Error reading eeprom image %s, %s\n", eeprom_file_name, strerror(errno));
        goto init_driver_end;
    }
    if (firmware_image_length &&
        fread(&init_info->data[eeprom_image_length], 1, firmware_image_length, f2)<firmware_image_length)
    {
        fprintf(stderr, "Error reading firmware image %s, %s\n", firmware_file_name, strerror(errno));
        goto init_driver_end;
    }
    if (init_file_length &&
        fread(&init_info->data[eeprom_image_length+firmware_image_length], 1, init_file_length, f3)<init_file_length)
    {
        fprintf(stderr, "Error reading init_file %s, %s\n", init_file_name, strerror(errno));
        goto init_driver_end;
    }

    rc = IPC_DeviceIoControl(adapter_name, TIWLN_SET_INIT_INFO, init_info, req_size, NULL, 0, NULL);

    /*Send configMge start command as the cli is started*/
    IPC_DeviceIoControl(adapter_name, TIWLN_DRIVER_STATUS_SET, &tmpData, sizeof(tiUINT32), NULL, 0, NULL);

init_driver_end:
    if (f1)
        fclose(f1);
    if (f2)
        fclose(f2);
    if (f3)
        fclose(f3);
    if (init_info)
        free(init_info);
#endif

	/*********************/
	/* Reset CLI events */
	/*******************/

	for (param.value = 0; param.value < IPC_EVENT_MAX; param.value++)
	{
		cmd_events_unregister(&param, 1);
	}

    return rc;
}

/* TRS:GAA separated Windows/Linux setup routines for readability */
#ifndef _WINDOWS
void init_extended_tools()
{
	/* Initialize IPC */
	ipc_initialize();

	/* Initialize ethernet utilities */
	ethernet_utils_init();

	/* Initialize wipp control */
	wipp_control_init();

	/* Initialize debug module task */
	debug_module_init();
}

void deinit_extended_tools()
{
	ipc_deinitialize();

	/* Deinitialize ethernet utilities */
	ethernet_utils_deinit();

	/* Deinitialize wipp control */
	wipp_control_deinit();

	/* Deinitializew debug module task */
	debug_module_deinit();
	}
#endif

#ifdef _WINDOWS
#endif
//TRS:GAA end of specific O/S init/deinit routines


// TICON.EXE main module
int main(int argc, char ** argv)
{
    int i;
    char *script_file = NULL;
    char  *eeprom_file_name = "/NVS/nvs_map.bin";
    char  *init_file_name = "/voice/tiwlan.ini";
    char *firmware_file_name = "/apps/firmware.bin";
    int stop_UI = 0;
    int bypass_supplicant = 0;
    ConParm_t param;

    // TRS:PGK -- To ensure that the data structure above is actually initialized to
    //            a NULL string.
    g_drv_name[0] = '\0';

#ifndef _WINDOWS
	/* TRS:AS for WM extended tools have to be initialized after parsing of the command line arguments*/
	init_extended_tools();
#endif /* __LINUX__*/
    if( argc > 1 )
    {
        i=1;
        if( argv[i][0] != '-' )
        {
            strcpy( g_drv_name, argv[i++] );
        }
        for( ;i < argc; i++ )
        {
            if( !strcmp(argv[i], "-h" ) || !strcmp(argv[i], "--help") )
                return print_usage(eeprom_file_name, init_file_name, firmware_file_name);
            else if(!strcmp(argv[i], "-f" ) )
            {
                firmware_file_name = argv[++i];
            }
            else if(!strcmp(argv[i], "-e") && (i+1<argc))
            {
                eeprom_file_name = argv[++i];
            }
            else if(!strcmp(argv[i], "-b"))
            {
                bypass_supplicant = 1;
            }
            else if(!strcmp(argv[i], "-i") && (i+1<argc))
            {
                init_file_name = argv[++i];
            }
            else if(!strcmp(argv[i], "-s" ) )
            {
                script_file = argv[++i];
            }
#ifdef _WINDOWS // TRS:AS for Windows only. -w switch allows to not disable WZC.
#endif /* ifdef _WINDOWS */
            else
            {
                fprintf(stderr, "ticon: unknown parameter '%s'\n", argv[i] );

#ifndef _WINDOWS
				deinit_extended_tools();
#endif /* ifdef __LINUX__*/

                return 0;
            }
        }
    }


    if( !g_drv_name[0] )
    {
		#ifndef _WINDOWS
			strcpy(g_drv_name, TIWLAN_DRV_NAME "0" );
		#else 
		#endif
    }

    /* TRS:PGK -- before this function call, do not use console_printf_terminal() */
	#ifdef _WINDOWS
	#endif /* ifdef _WINDOWS */
    //console_printf_terminal("ticon: g_drv_name = %s\n", g_drv_name );

    // TRS:PGK -- From this point on, use console_printf_terminal()

    if (init_driver(g_drv_name, eeprom_file_name, init_file_name, firmware_file_name) != 0)
	{
		deinit_extended_tools();
        return -1;
	}

#ifndef _WINDOWS
	/* Initialize g_tester module */
	g_tester_init();
#endif /* __LINUX__ */

    if (!bypass_supplicant)
    {
        console_printf_terminal("Starting up supplicant...\n" );
        TI_StartSM(g_id_adapter);
    }

    /* ----------------------------------------------------------- */
    init_console_menu();

    init_scan_params();

#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */

    if( script_file )
    {
        stop_UI = consoleRunScript (script_file);
    }

    if( !stop_UI )
        consoleStart();

#ifndef _WINDOWS
	/* Deinitialize g_tester module */
	g_tester_deinit();
#endif /* __LINUX__ */

    // TRS:PGK -- NOW it is good to assume that the events might have been
    //  registered in the past.
    for (param.value = 0; param.value < IPC_EVENT_MAX; param.value++)
    {
        cmd_events_unregister(&param, 1);
    }


    if (!bypass_supplicant)
    {
        console_printf_terminal("Stop supplicant manager...\n" );
        TI_StopSM(g_id_adapter);
    }
#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */

    console_printf_terminal("De-init the adapter...\n" );
    TI_AdapterDeinit(g_id_adapter);

	deinit_extended_tools();

    printf("\nLeaving ticon\n");  //TRS:MEB added printf

    return 1;
}

