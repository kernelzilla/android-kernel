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

#ifndef G_TESTER_H
#define G_TESTER_H

#define G_TESTER_CMD_GROUP_CLI			(0x1000)
#define G_TESTER_CMD_GROUP_GWSI			(0x2000)
#define G_TESTER_CMD_GROUP_GENERAL		(0x4000)

                                  
#define G_TESTER_IS_CLI_GROUP_CMD(_op)	((_op & G_TESTER_CMD_GROUP_CLI) != 0)
#define G_TESTER_IS_GWSI_GROUP_CMD(_op)	((_op & G_TESTER_CMD_GROUP_GWSI) != 0)
#define G_TESTER_IS_GENERAL_GROUP_CMD(_op)	((_op & G_TESTER_CMD_GROUP_GENERAL) != 0)

#define G_TESTER_GENERAL_CMD_RUN_CMD	(G_TESTER_CMD_GROUP_GENERAL | 0x0001)
#define G_TESTER_GENERAL_CMD_GET_INIT_T	(G_TESTER_CMD_GROUP_GENERAL | 0x0002)

#define G_TESTER_GWSI_CMD_INITIALIZE	(G_TESTER_CMD_GROUP_GWSI | 0x0060)
#define G_TESTER_GWSI_CMD_CONFIG	    (G_TESTER_CMD_GROUP_GWSI | 0x0061)
#define G_TESTER_GWSI_CMD_PLT		    (G_TESTER_CMD_GROUP_GWSI | 0x0062)
#define G_TESTER_GWSI_CMD_RELEASE		(G_TESTER_CMD_GROUP_GWSI | 0x0063)


#define G_TESTER_CLI_CMD_DRIVER__START                            (G_TESTER_CMD_GROUP_CLI | 0x0001)
#define G_TESTER_CLI_CMD_DRIVER__STOP                             (G_TESTER_CMD_GROUP_CLI | 0x0002)
#define G_TESTER_CLI_CMD_DRIVER__STATUS                           (G_TESTER_CMD_GROUP_CLI | 0x0003)
#define G_TESTER_CLI_CMD_ROOT__ABOUT                              (G_TESTER_CMD_GROUP_CLI | 0x0004)
#define G_TESTER_CLI_CMD_CONNECTION__BSSID_LIST                   (G_TESTER_CMD_GROUP_CLI | 0x0010)
#define G_TESTER_CLI_CMD_CONNECTION__CONNECT                      (G_TESTER_CMD_GROUP_CLI | 0x0013)
#define G_TESTER_CLI_CMD_CONNECTION__DISASSOCIATE                 (G_TESTER_CMD_GROUP_CLI | 0x0014)
#define G_TESTER_CLI_CMD_PRIVACY__AUTHENTICATION                  (G_TESTER_CMD_GROUP_CLI | 0x0030)
#define G_TESTER_CLI_CMD_PRIVACY__EXC__NETWORKEAP                 (G_TESTER_CMD_GROUP_CLI | 0x0031)
#define G_TESTER_CLI_CMD_PRIVACY__ENCRYPTION                      (G_TESTER_CMD_GROUP_CLI | 0x0032)
#define G_TESTER_CLI_CMD_PRIVACY__KEYTYPE                         (G_TESTER_CMD_GROUP_CLI | 0x0033)
#define G_TESTER_CLI_CMD_PRIVACY__MIXEDMODE                       (G_TESTER_CMD_GROUP_CLI | 0x0034)
#define G_TESTER_CLI_CMD_PRIVACY__CREDENTIALS                     (G_TESTER_CMD_GROUP_CLI | 0x0035)
#define G_TESTER_CLI_CMD_PRIVACY__WEP__ADD                        (G_TESTER_CMD_GROUP_CLI | 0x0036)
#define G_TESTER_CLI_CMD_PRIVACY__WEP__REMOVE                     (G_TESTER_CMD_GROUP_CLI | 0x0037)
#define G_TESTER_CLI_CMD_SCAN__START                              (G_TESTER_CMD_GROUP_CLI | 0x0040)
#define G_TESTER_CLI_CMD_SCAN__STOP                               (G_TESTER_CMD_GROUP_CLI | 0x0041)
#define G_TESTER_CLI_CMD_SCAN__CONFIGAPP__GLOBAL                  (G_TESTER_CMD_GROUP_CLI | 0x0042)
#define G_TESTER_CLI_CMD_SCAN__CONFIGAPP__CHANNEL                 (G_TESTER_CMD_GROUP_CLI | 0x0043)
#define G_TESTER_CLI_CMD_SCAN__CONFIGAPP__CLEAR                   (G_TESTER_CMD_GROUP_CLI | 0x0044)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__GLOABAL              (G_TESTER_CMD_GROUP_CLI | 0x0045)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__MISC           (G_TESTER_CMD_GROUP_CLI | 0x0046)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__CHANNEL        (G_TESTER_CMD_GROUP_CLI | 0x0047)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__TRACK          (G_TESTER_CMD_GROUP_CLI | 0x0048)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__DISCOVERY      (G_TESTER_CMD_GROUP_CLI | 0x0049)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BAND__IMMEDIATE      (G_TESTER_CMD_GROUP_CLI | 0x004a)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__CLEAR                (G_TESTER_CMD_GROUP_CLI | 0x004b)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__STORE                (G_TESTER_CMD_GROUP_CLI | 0x004c)
#define G_TESTER_CLI_CMD_EVENTS__REGISTER                         (G_TESTER_CMD_GROUP_CLI | 0x0050)
#define G_TESTER_CLI_CMD_EVENTS__UNREGISTER                       (G_TESTER_CMD_GROUP_CLI | 0x0051)
#define G_TESTER_CLI_CMD_REPORT__LEVEL                            (G_TESTER_CMD_GROUP_CLI | 0x0060)
#define G_TESTER_CLI_CMD_REPORT__SET                              (G_TESTER_CMD_GROUP_CLI | 0x0061)
#define G_TESTER_CLI_CMD_DEBUG__PRINT                             (G_TESTER_CMD_GROUP_CLI | 0x0062)
#define G_TESTER_CLI_CMD_MANAGEMENT__RATE                         (G_TESTER_CMD_GROUP_CLI | 0x0070)
#define G_TESTER_CLI_CMD_MANAGEMENT__MODE                         (G_TESTER_CMD_GROUP_CLI | 0x0071)
#define G_TESTER_CLI_CMD_MANAGEMENT__PREAMBLE                     (G_TESTER_CMD_GROUP_CLI | 0x0072)
#define G_TESTER_CLI_CMD_MANAGEMENT__CHANNEL                      (G_TESTER_CMD_GROUP_CLI | 0x0073)
#define G_TESTER_CLI_CMD_MANAGEMENT__FRAG                         (G_TESTER_CMD_GROUP_CLI | 0x0074)
#define G_TESTER_CLI_CMD_MANAGEMENT__RTS                          (G_TESTER_CMD_GROUP_CLI | 0x0075)
#define G_TESTER_CLI_CMD_MANAGEMENT__SLOT                         (G_TESTER_CMD_GROUP_CLI | 0x0076)
#define G_TESTER_CLI_CMD_MANAGEMENT__SIGNAL                       (G_TESTER_CMD_GROUP_CLI | 0x0077)
#define G_TESTER_CLI_CMD_MANAGEMENT__TX_POWER_LEVEL               (G_TESTER_CMD_GROUP_CLI | 0x0078)
#define G_TESTER_CLI_CMD_MANAGEMENT__SSID                         (G_TESTER_CMD_GROUP_CLI | 0x007c)
#define G_TESTER_CLI_CMD_POWER__SET_POWER_MODE                    (G_TESTER_CMD_GROUP_CLI | 0x0090)
#define G_TESTER_CLI_CMD_ROAMING__ENABLE                          (G_TESTER_CMD_GROUP_CLI | 0x00a0)
#define G_TESTER_CLI_CMD_ROAMING__LOW_PASS_FILTER                 (G_TESTER_CMD_GROUP_CLI | 0x00a1)
#define G_TESTER_CLI_CMD_ROAMING__QUALITY_THRESHOLD               (G_TESTER_CMD_GROUP_CLI | 0x00a2)
#define G_TESTER_CLI_CMD_ROAMING__SET                             (G_TESTER_CMD_GROUP_CLI | 0x00a3)

#define G_TESTER_CLI_CMD_CONNECTION__STATUS                       (G_TESTER_CMD_GROUP_CLI | 0x0106)
#define G_TESTER_CLI_CMD_CONNECTION__FULL_BSSID_LIST              (G_TESTER_CMD_GROUP_CLI | 0x0107)
#define G_TESTER_CLI_CMD_MANAGEMENT__INFO                         (G_TESTER_CMD_GROUP_CLI | 0x0110)
#define G_TESTER_CLI_CMD_MANAGEMENT__DRIVERSTATE                  (G_TESTER_CMD_GROUP_CLI | 0x0111)
#define G_TESTER_CLI_CMD_MANAGEMENT__TX_POWER_DBM                 (G_TESTER_CMD_GROUP_CLI | 0x0114)
#define G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__D_ENABLEDISABLE   (G_TESTER_CMD_GROUP_CLI | 0x0115)
#define G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__H_ENABLEDISABLE   (G_TESTER_CMD_GROUP_CLI | 0x0116)
#define G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__D_COUNTRY_2_4IE   (G_TESTER_CMD_GROUP_CLI | 0x0117)
#define G_TESTER_CLI_CMD_MANAGEMENT__802_11D_H__D_COUNTRY_5IE     (G_TESTER_CMD_GROUP_CLI | 0x0118)
#define G_TESTER_CLI_CMD_MANAGEMENT__ANTENNA__DIVERSITYPARAMS     (G_TESTER_CMD_GROUP_CLI | 0x0119)
#define G_TESTER_CLI_CMD_MANAGEMENT__BEACON__SET_BEACON_FILTER_MODE (G_TESTER_CMD_GROUP_CLI | 0x011a)
#define G_TESTER_CLI_CMD_MANAGEMENT__ADVANCED__DRAFT              (G_TESTER_CMD_GROUP_CLI | 0x011b)
#define G_TESTER_CLI_CMD_MANAGEMENT__ADVANCED__SUPPORTED_RATES    (G_TESTER_CMD_GROUP_CLI | 0x011c)
#define G_TESTER_CLI_CMD_SHOW__STATISTICS                         (G_TESTER_CMD_GROUP_CLI | 0x011d)
#define G_TESTER_CLI_CMD_SHOW__TX_STATISTICS                      (G_TESTER_CMD_GROUP_CLI | 0x011e)
#define G_TESTER_CLI_CMD_SHOW__ADVANCED                           (G_TESTER_CMD_GROUP_CLI | 0x011f)
#define G_TESTER_CLI_CMD_PRIVACY__EAP                             (G_TESTER_CMD_GROUP_CLI | 0x0121)
#define G_TESTER_CLI_CMD_PRIVACY__PSKPASSPHRASE                   (G_TESTER_CMD_GROUP_CLI | 0x0126)
#define G_TESTER_CLI_CMD_PRIVACY__CERTIFICATE                     (G_TESTER_CMD_GROUP_CLI | 0x0127)
#define G_TESTER_CLI_CMD_PRIVACY__WPA_OPTIONS                     (G_TESTER_CMD_GROUP_CLI | 0x0128)
#define G_TESTER_CLI_CMD_PRIVACY__EXC__CONFIGURE                  (G_TESTER_CMD_GROUP_CLI | 0x012b)
#define G_TESTER_CLI_CMD_SCAN__CONFIGAPP__DISPLAY                 (G_TESTER_CMD_GROUP_CLI | 0x0132)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__DISPLAY              (G_TESTER_CMD_GROUP_CLI | 0x0139)
#define G_TESTER_CLI_CMD_SCAN__CONFIGPOLICY__BSSLIST              (G_TESTER_CMD_GROUP_CLI | 0x013c)
#define G_TESTER_CLI_CMD_ROAMING__GET                             (G_TESTER_CMD_GROUP_CLI | 0x0141)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__TX_RETRY            (G_TESTER_CMD_GROUP_CLI | 0x0143)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__BSS_LOSS            (G_TESTER_CMD_GROUP_CLI | 0x0144)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__TX_RATE_THRESHOLD   (G_TESTER_CMD_GROUP_CLI | 0x0145)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__LOW_RSSI_THRESHOLD  (G_TESTER_CMD_GROUP_CLI | 0x0146)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__LOW_SNR_THRESHOLD   (G_TESTER_CMD_GROUP_CLI | 0x0147)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__LOW_QUALITY_FOR_SCAN (G_TESTER_CMD_GROUP_CLI | 0x0148)
#define G_TESTER_CLI_CMD_ROAMING__THRESHOLDS__NORMAL_QUALITY_FOR_SCAN (G_TESTER_CMD_GROUP_CLI | 0x0149)
#define G_TESTER_CLI_CMD_QOS__UPSD__ADD                           (G_TESTER_CMD_GROUP_CLI | 0x014a)
#define G_TESTER_CLI_CMD_QOS__UPSD__GET                           (G_TESTER_CMD_GROUP_CLI | 0x014b)
#define G_TESTER_CLI_CMD_QOS__UPSD__DELETE                        (G_TESTER_CMD_GROUP_CLI | 0x014c)
#define G_TESTER_CLI_CMD_QOS__UPSD__AP_PARAMS                     (G_TESTER_CMD_GROUP_CLI | 0x014d)
#define G_TESTER_CLI_CMD_QOS__UPSD__AP_CAPABILITIES               (G_TESTER_CMD_GROUP_CLI | 0x014e)
#define G_TESTER_CLI_CMD_QOS__UPSD__AC_STATUS                     (G_TESTER_CMD_GROUP_CLI | 0x014f)
#define G_TESTER_CLI_CMD_QOS__UPSD__MEDIUM_USAGE                  (G_TESTER_CMD_GROUP_CLI | 0x0150)
#define G_TESTER_CLI_CMD_QOS__UPSD__PHY_RATE                      (G_TESTER_CMD_GROUP_CLI | 0x0151)
#define G_TESTER_CLI_CMD_QOS__UPSD__DESIRED_PS_MODE               (G_TESTER_CMD_GROUP_CLI | 0x0152)
#define G_TESTER_CLI_CMD_QOS__CLASSIFIER__TXCLASSIFIER            (G_TESTER_CMD_GROUP_CLI | 0x0153)
#define G_TESTER_CLI_CMD_QOS__CLASSIFIER__INSERT                  (G_TESTER_CMD_GROUP_CLI | 0x0154)
#define G_TESTER_CLI_CMD_QOS__CLASSIFIER__REMOVE                  (G_TESTER_CMD_GROUP_CLI | 0x0155)
#define G_TESTER_CLI_CMD_QOS__QOSPARAMS                           (G_TESTER_CMD_GROUP_CLI | 0x0156)
#define G_TESTER_CLI_CMD_QOS__POLL_AP_PACKETS                     (G_TESTER_CMD_GROUP_CLI | 0x0157)
#define G_TESTER_CLI_CMD_QOS__RX_TIMEOUT                          (G_TESTER_CMD_GROUP_CLI | 0x0158)
#define G_TESTER_CLI_CMD_POWER__SET_POWERSAVE_POWERLEVEL          (G_TESTER_CMD_GROUP_CLI | 0x015a)
#define G_TESTER_CLI_CMD_POWER__TRAFFIC_THRESHOLDS                (G_TESTER_CMD_GROUP_CLI | 0x015b)
#define G_TESTER_CLI_CMD_POWER__ENABLE                            (G_TESTER_CMD_GROUP_CLI | 0x015c)
#define G_TESTER_CLI_CMD_POWER__DISABLE                           (G_TESTER_CMD_GROUP_CLI | 0x015d)
#define G_TESTER_CLI_CMD_FILE__LOAD                               (G_TESTER_CMD_GROUP_CLI | 0x0162)
#define G_TESTER_CLI_CMD_BT_COEXSISTANCE__ENABLE                  (G_TESTER_CMD_GROUP_CLI | 0x0163)
#define G_TESTER_CLI_CMD_BT_COEXSISTANCE__RATE                    (G_TESTER_CMD_GROUP_CLI | 0x0164)
#define G_TESTER_CLI_CMD_BT_COEXSISTANCE__CONFIG                  (G_TESTER_CMD_GROUP_CLI | 0x0165)
#define G_TESTER_CLI_CMD_BT_COEXSISTANCE__STATUS                  (G_TESTER_CMD_GROUP_CLI | 0x0166)
#define G_TESTER_CLI_CMD_MEASUREMENT__ENABLE                      (G_TESTER_CMD_GROUP_CLI | 0x0167)
#define G_TESTER_CLI_CMD_MEASUREMENT__DISABLE                     (G_TESTER_CMD_GROUP_CLI | 0x0168)
#define G_TESTER_CLI_CMD_MEASUREMENT__MAX_DURATION                (G_TESTER_CMD_GROUP_CLI | 0x0169)
#define G_TESTER_CLI_CMD_REPORT__ADD                              (G_TESTER_CMD_GROUP_CLI | 0x016b)
#define G_TESTER_CLI_CMD_REPORT__CLEAR                            (G_TESTER_CMD_GROUP_CLI | 0x016c)
#define G_TESTER_CLI_CMD_DEBUG__REGISTER                          (G_TESTER_CMD_GROUP_CLI | 0x016e)
#define G_TESTER_CLI_CMD_DEBUG__BUFFER                            (G_TESTER_CMD_GROUP_CLI | 0x0170)
#define G_TESTER_CLI_CMD_ROOT__QUIT                               (G_TESTER_CMD_GROUP_CLI | 0x0172)

#define G_TESTER_CLI_CMD_PLT__REGISTER__READ                      (G_TESTER_CMD_GROUP_CLI | 0x0173)
#define G_TESTER_CLI_CMD_PLT__REGISTER__WRITE                     (G_TESTER_CMD_GROUP_CLI | 0x0174)
#define G_TESTER_CLI_CMD_PLT__RX_PER__START                       (G_TESTER_CMD_GROUP_CLI | 0x0175)
#define G_TESTER_CLI_CMD_PLT__RX_PER__STOP                        (G_TESTER_CMD_GROUP_CLI | 0x0176)
#define G_TESTER_CLI_CMD_PLT__RX_PER__CLEAR                       (G_TESTER_CMD_GROUP_CLI | 0x0177)
#define G_TESTER_CLI_CMD_PLT__RX_PER__GET_RESULTS                 (G_TESTER_CMD_GROUP_CLI | 0x0178)
#define G_TESTER_CLI_CMD_PLT__TX__CW                              (G_TESTER_CMD_GROUP_CLI | 0x0179)
#define G_TESTER_CLI_CMD_PLT__TX__CONTINUES                       (G_TESTER_CMD_GROUP_CLI | 0x017a)
#define G_TESTER_CLI_CMD_PLT__TX__STOP                            (G_TESTER_CMD_GROUP_CLI | 0x017b)
#define G_TESTER_CLI_CMD_PLT__MIB__READ                           (G_TESTER_CMD_GROUP_CLI | 0x017c)
#define G_TESTER_CLI_CMD_PLT__MIB__WRITE                          (G_TESTER_CMD_GROUP_CLI | 0x017d)

#define G_TESTER_CLI_CMD_PRIVACY__WEP__GET                        (G_TESTER_CMD_GROUP_CLI | 0x017e)

#define G_TESTER_CLI_CMD_PLT__CALIBRATION__RX                     (G_TESTER_CMD_GROUP_CLI | 0x017f)
#define G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__START              (G_TESTER_CMD_GROUP_CLI | 0x0180)
#define G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__STOP               (G_TESTER_CMD_GROUP_CLI | 0x0181)
#define G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__GAIN_GET           (G_TESTER_CMD_GROUP_CLI | 0x0182)
#define G_TESTER_CLI_CMD_PLT__CALIBRATION__TX__GAIN_ADJUST        (G_TESTER_CMD_GROUP_CLI | 0x0183)
#define G_TESTER_CLI_CMD_PLT__CALIBRATION__GET_NVS_BUFFER         (G_TESTER_CMD_GROUP_CLI | 0x0184)
#define G_TESTER_CLI_CMD_PLT_RADIO_TUNE                           (G_TESTER_CMD_GROUP_CLI | 0x0185)

void g_tester_init(void);
void g_tester_deinit(void);					
	  
void g_tester_receive_event(unsigned char event_index);
unsigned char g_tester_check_command(unsigned char *input_string);

#endif /* G_TESTER_H */

