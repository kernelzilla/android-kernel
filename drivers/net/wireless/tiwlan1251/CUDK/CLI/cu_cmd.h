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

#ifndef CU_CMD_H
#define CU_CMD_H

#ifndef _WINDOWS
#include <unistd.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/wireless.h>
#endif /* __LINUX__ */

#ifdef _WINDOWS
#endif

#include "paramOut.h"
#include "scanMngrTypes.h"

#ifdef EXC_MODULE_INCLUDED
#include "TI_AdapterEXC.h"
#endif /*EXC_MODULE_INCLUDED*/

#include "TI_AdapterApiC.h"

#include "console.h"

#ifndef _WINDOWS
#define stricmp strcasecmp
#endif

#ifdef EXC_MODULE_INCLUDED
# include "cu_cmd_exc.h"
#else
# define INCLUDE_EXC_TYPE_NAMES
#endif

extern scan_Params_t appScanParams;

void init_scan_params(void);

void cmd_show_status(ConParm_t parm[], U16 nParms);
void cmd_show_site_table(ConParm_t parm[], U16 nParms);
void cmd_connect(ConParm_t parm[], U16 nParms);
void cmd_disassociate(ConParm_t parm[], U16 nParms);
void cmd_show_advanced_params(ConParm_t parm[], U16 nParms);
void cmd_show_statistics(ConParm_t parm[], U16 nParms);
void cmd_show_tx_statistics(ConParm_t parm[], U16 nParms);
void cmd_show_power_consumption_stats(ConParm_t parm[]);
void cmd_show_about(ConParm_t parm[], U16 nParms);
void cmd_modify_ssid(ConParm_t parm[], U16 nParms);
void cmd_bssid_list(ConParm_t parm[], U16 nParms);
void cmd_Full_bssid_list(ConParm_t parm[], U16 nParms);
void cmd_FullPrimaryBbssid(ConParm_t parm[], U16 nParms);


void cmd_Scan_Start(ConParm_t parm[], U16 nParms);
void cmd_Scan_Stop(ConParm_t parm[], U16 nParms);
void cmd_Scan_app_global_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_app_channel_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_app_clear(ConParm_t parm[], U16 nParms);
void cmd_Scan_app_display(ConParm_t parm[], U16 nParms);

void cmd_Scan_policy_global_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_band_global_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_band_channel_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_band_track_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_band_discover_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_band_immed_config(ConParm_t parm[], U16 nParms);
void cmd_Scan_policy_display(ConParm_t parm[], U16 nParms);
void cmd_Scan_print_band( int i );
void cmd_Scan_print_method( scan_Method_t* scanMethod );
void cmd_Scan_print_basic_method( scan_basicMethodParams_t* basicMethodParams );
void cmd_Scan_print_triggered_method( scan_TidTriggeredMethodParams_t* triggeredMethodParams );
void cmd_Scan_print_sps_method( scan_SPSMethodParams_t* spsMethodParams );
void cmd_Scan_policy_clear(ConParm_t parm[], U16 nParms);
void cmd_Scan_policy_store(ConParm_t parm[], U16 nParms);
void cmd_Scan_get_bss_list(ConParm_t parm[], U16 nParms);

void cmd_get_selected_bssid_info(ConParm_t parm[], U16 nParms);
void cmd_get_driver_state(ConParm_t parm[], U16 nParms);

void cmd_get_rsii_level(ConParm_t parm[], U16 nParms);
void cmd_get_snr_ratio(ConParm_t parm[], U16 nParms);

void cmd_set_clsfr_type (ConParm_t parm[], U16 nParms);
void cmd_insert_clsfr_entry (ConParm_t parm[], U16 uParms);
void cmd_remove_clsfr_entry (ConParm_t parm[], U16 uParms);

void cmd_set_qos_params(ConParm_t parm[], U16 nParms);
void cmd_set_dtag_to_ac_mapping_table(ConParm_t parm[], U16 nParms);
void cmd_set_vad(ConParm_t parm[], U16 nParms);
void cmd_config_tx_classifier(ConParm_t parm[], U16 nParms);
void cmd_poll_ap_packets(ConParm_t parm[], U16 nParms);
void cmd_set_rxTimeOut_params(ConParm_t parm[], U16 nParms);
void cmd_enable_rx_data_filters(ConParm_t parm[], U16 nParms);
void cmd_disable_rx_data_filters(ConParm_t parm[], U16 nParms);
void cmd_get_rx_data_filters_statistics(ConParm_t parm[], U16 nParms);
void cmd_add_rx_data_filter(ConParm_t parm[], U16 nParms);
void cmd_remove_rx_data_filter(ConParm_t parm[], U16 nParms);
void cmd_MaxRxLifetime_params(ConParm_t parm[], U16 nParms);

void cmd_add_tspec(ConParm_t parm[], U16 nParms);
void cmd_get_tspec_params(ConParm_t parm[], U16 nParms);
void cmd_delete_tspec(ConParm_t parm[], U16 nParms);
void cmd_get_ap_qos_params(ConParm_t parm[], U16 nParms);
void cmd_get_ap_qos_capabilities(ConParm_t parm[], U16 nParms);
void cmd_get_ac_status(ConParm_t parm[], U16 nParms);
int  parseBssidIe(OS_802_11_BSSID_EX * bssid);
void cmd_get_desired_ps_mode(ConParm_t parm[], U16 nParms);
void cmd_medium_usage_threshold(ConParm_t parm[], U16 nParms);
void cmd_phy_rate_threshold(ConParm_t parm[], U16 nParms);

void cmd_traffic_intensity_threshold(ConParm_t parm[], U16 nParms);
void cmd_enable_traffic_events(ConParm_t parm[], U16 nParms);
void cmd_disable_traffic_events(ConParm_t parm[], U16 nParms);

void cmd_events_config(ConParm_t parm[], U16 nParms);

void cmd_show_regdomain_table(ConParm_t parm[], U16 nParms);
/*void cmd_net_network_in_use(ConParm_t parm[], U16 nParms);         (not in use) */
void cmd_net_current_regdomain(ConParm_t parm[], U16 nParms);
void cmd_enableDisable_802_11d(ConParm_t parm[], U16 nParms);
void cmd_enableDisable_802_11h(ConParm_t parm[], U16 nParms);
void cmd_d_Country_2_4Ie(ConParm_t parm[], U16 nParms);
void cmd_d_Country_5Ie(ConParm_t parm[], U16 nParms);
void cmd_DFS_range(ConParm_t parm[], U16 nParms);



void cmd_modify_channel(ConParm_t parm[], U16 nParms);
void cmd_modify_rate(ConParm_t parm[], U16 nParms);
void cmd_show_tx_power_level_table(ConParm_t parm[], U16 nParms);
void cmd_tx_power_dbm(ConParm_t parm[], U16 nParms);
void cmd_modify_frag_threshold(ConParm_t parm[], U16 nParms);
void cmd_modify_rts_threshold(ConParm_t parm[], U16 nParms);
void cmd_modify_preamble(ConParm_t parm[], U16 nParms);
void cmd_modify_short_retry(ConParm_t parm[], U16 nParms);
void cmd_modify_long_retry(ConParm_t parm[], U16 nParms);
void cmd_modify_short_slot(ConParm_t parm[], U16 nParms);
void cmd_modify_tx_antenna(ConParm_t parm[], U16 nParms);
void cmd_modify_rx_antenna(ConParm_t parm[], U16 nParms);
void cmd_modify_antenna_diversity(ConParm_t parm[], U16 nParms);

void cmd_modify_4x_state(ConParm_t parm[], U16 nParms);
void cmd_modify_ext_rates_ie(ConParm_t parm[], U16 nParms);
void cmd_modify_supported_rates(ConParm_t parm[], U16 nParms);
void cmd_modify_ctsToSelf(ConParm_t parm[], U16 nParms);


void cmd_debug_level(ConParm_t parm[], U16 nParms);
void cmd_hw_register(ConParm_t parm[], U16 nParms);
void cmd_debug_driver_print(ConParm_t parm[], U16 nParms);
void cmd_debug_buffer_put(ConParm_t parm[], U16 nParms);
#ifdef DRIVER_PROFILING
void cmd_profile_report(ConParm_t parm[], U16 nParms);
void cmd_profile_cpu_estimator_command(ConParm_t parm[], U16 nParms);
#endif

void cmd_report_set(ConParm_t parm[], U16 nParms);
void cmd_report_clear(ConParm_t parm[], U16 nParms);
void cmd_report_add(ConParm_t parm[], U16 nParms);
void cmd_report_severity_level(ConParm_t parm[], U16 nParms);
void cmd_report_severity_table(ConParm_t parm[], U16 nParms);
void cmd_report_os_dbg_state(ConParm_t parm[], U16 nParms);

void cmd_modify_bss_type(ConParm_t parm[], U16 nParms);

void cmd_init_driver(ConParm_t parm[], U16 nParms);
void cmd_start_driver(ConParm_t parm[], U16 nParms);
void cmd_stop_driver(ConParm_t parm[], U16 nParms);

void cmd_privacy_auth(ConParm_t parm[], U16 nParms);
void cmd_privacy_eap(ConParm_t parm[], U16 nParms);
void cmd_privacy_encrypt(ConParm_t parm[], U16 nParms);
void cmd_privacy_credent(ConParm_t parm[], U16 nParms);
void cmd_privacy_PSKPassphrase(ConParm_t parm[], U16 nParms);
void cmd_privacy_certificate(ConParm_t parm[], U16 nParms);
void cmd_privacy_addkey(ConParm_t parm[], U16 nParms);
void cmd_privacy_removekey(ConParm_t parm[], U16 nParms);
void cmd_privacy_cckm(ConParm_t parm[], U16 nParms);
void cmd_privacy_wpa_options(ConParm_t parm[], U16 nParms);
void cmd_privacy_getdefaultkey(ConParm_t parm[], U16 nParms);

void cmd_file_load(ConParm_t parm[], U16 nParms);

void cmd_set_power_mode(ConParm_t parm[], U16 nParms);
void cmd_set_PowerSave_PowerLevel(ConParm_t parm[], U16 nParms);
void cmd_set_Default_PowerLevel(ConParm_t parm[], U16 nParms);
void cmd_set_DozeModeInAutoPowerLevel(ConParm_t parm[], U16 nParms);
void cmd_set_min_power_level_boundary(ConParm_t parm[], U16 nParms);

void cmd_events_register(ConParm_t parm[], U16 nParms);
void cmd_events_unregister(ConParm_t parm[], U16 nParms);

void cmd_bt_coe_enable(ConParm_t parm[], U16 nParms);
void cmd_bt_coe_rate(ConParm_t parm[], U16 nParms);
void cmd_bt_coe_config(ConParm_t parm[], U16 nParms);
void cmd_bt_coe_get_status(ConParm_t parm[], U16 nParms);

void cmd_privacy_key_type(ConParm_t parm[], U16 nParms);
void cmd_privacy_mixed_mode(ConParm_t parm[], U16 nParms);



void cmd_Roaming_enable(ConParm_t parm[], U16 nParms);
void cmd_Roaming_disable(ConParm_t parm[], U16 nParms);
void cmd_Roaming_lowPassFilter(ConParm_t parm[], U16 nParms);
void cmd_Roaming_qualityIndicator(ConParm_t parm[], U16 nParms);
void cmd_Roaming_getConfParams(ConParm_t parm[], U16 nParms);

void cmd_Roaming_dataRetryThreshold(ConParm_t parm[], U16 nParms);
void cmd_Roaming_numExpectedTbttForBSSLoss(ConParm_t parm[], U16 nParms);
void cmd_Roaming_txRateThreshold(ConParm_t parm[], U16 nParms);
void cmd_Roaming_lowSnrThreshold(ConParm_t parm[], U16 nParms);
void cmd_Roaming_lowRssiThreshold(ConParm_t parm[], U16 nParms);
void cmd_Roaming_lowQualityForBackgroungScanCondition(ConParm_t parm[], U16 nParms);
void cmd_Roaming_normalQualityForBackgroungScanCondition(ConParm_t parm[], U16 nParms);
void cmd_Roaming_rssiFilterWeight(ConParm_t parm[], U16 nParms);
void cmd_Roaming_snrFilterWeight(ConParm_t parm[], U16 nParms);

void cmd_Beacon_Filter_Set_Desired_State(ConParm_t parm[], U16 nParms);
void cmd_Beacon_Filter_Get_Desired_State(ConParm_t parm[], U16 nParms);

void cmd_PLT_RegisterRead(ConParm_t parm[], U16 nParms);
void cmd_PLT_RegisterWrite(ConParm_t parm[], U16 nParms);
void cmd_PLT_RxPerStart(ConParm_t parm[], U16 nParms);
void cmd_PLT_RxPerStop(ConParm_t parm[], U16 nParms);
void cmd_PLT_RxPerClear(ConParm_t parm[], U16 nParms);
void cmd_PLT_RxPerGet(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxCW(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxContinues(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxStop(ConParm_t parm[], U16 nParms);
void cmd_PLT_MIB_CounterTable(ConParm_t parm[], U16 nParms);
void cmd_PLT_MIB_StationID(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxCalGainGet(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxCalGainAdjust(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxCalStart(ConParm_t parm[], U16 nParms);
void cmd_PLT_TxCalStop(ConParm_t parm[], U16 nParms);
void cmd_PLT_RxTxCalNVSUpdateBuffer(ConParm_t parm[], U16 nParms);
void cmd_PLT_RxCal(ConParm_t parm[], U16 nParms);
void cmd_PLT_RadioTune(ConParm_t parm[], U16 nParms);
void cmd_get_arpIpTable(ConParm_t parm[], U16 nParms);
void cmd_get_GroupAddressTable(ConParm_t parm[], U16 nParms);

#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */

U8* str2MACAddr(char *str, U8 *mac);
UINT8 Freq2Chan(UINT32 freq);
unsigned int char_2_hexa( char c );

#ifdef DEBUG
# define CHK_NULL(p)    ((p)) ? (void) 0 : fprintf(stderr, "\nfailed: '%s', file %s, line %d\n", #p, __FILE__, __LINE__);
# define CHK(p)        ((!p)) ? (void) 0 : fprintf(stderr, "\nfailed: '%s', file %s, line %d\n", #p, __FILE__, __LINE__);
#else
# define CHK(p)        (p)
# define CHK_NULL(p)    (p)
#endif

#endif /* CU_CMD_H */

