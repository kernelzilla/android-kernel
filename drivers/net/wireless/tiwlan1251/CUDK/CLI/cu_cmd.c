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

#ifdef _WINDOWS
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef _WINDOWS
	#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/wireless.h>

	#include "tnetwCommon.h"
#endif /* __LINUX__ */

#include "g_tester.h"
#include "wspVer.h"

#include "paramOut.h"
#include "console.h"
#include "ticon.h"
#include "cu_cmd.h"
#include "linux_ioctl_common.h"
#include "802_11Defs.h"

#ifndef _T
#define _T(a)    a
#endif

#ifndef _WINDOWS
TI_HANDLE   g_id_adapter = 0;
#endif

#define MAX_SSID_LEN 32
#define CLI_NUM_OF_TX_CLASFR_CON 4

scan_Params_t appScanParams;
scan_Policy_t scanPolicy;

/*** Roaming Manager configuration parameters ***/
roamingMngrConfigParams_t   roamingMngrConfigParams;

static tiUINT32 events_mask = 0; // TRS:PGK

#define NET_BASIC_MASK      0x80    /* defined in common/src/utils/utils.c */

#define IS_BASIC_RATE(a)    ((a) & NET_BASIC_MASK)

#define RATE_2_MBPS(a)    ((float)((a) & (NET_BASIC_MASK-1))/2)

#define GET_NAME_BY_VALUE(arr, value)    get_name_by_value(arr, SIZE_ARR(arr), value)

#define CHAN_FREQ_TABLE_SIZE        (sizeof(ChanFreq) / sizeof(struct CHAN_FREQ))

#define RATE_TABLE_SIZE             (sizeof(rate2Str) / sizeof(named_value_t))

#define ET_TABLE_SIZE               (sizeof(EtEvent2Str) / sizeof(named_value_t))

/* EarlyTermination DISABLE mode - Should be same as in scan.h */
#define SCAN_ET_COND_DISABLE 0

#define MAX_PSK_STRING_LENGTH       63

#define MIN_PSK_STRING_LENGTH       8

#define PSK_HEXA_LENGTH       64

#define PSK_BUFF_LEN 65

static void get_bssid_list(ConParm_t parm[], U16 nParms, BOOL fullBssidList , OS_802_11_BSSID_EX *pBssid);

static BOOL is_value_rate (tiUINT32 rate)
{

   switch (rate)
   {
   case 1:
   case 2:
   case 5:
   case 6:
   case 9:
   case 11:
   case 12:
   case 18:
   case 22:
   case 24:
   case 36:
   case 48:
   case 54:
      return (TRUE);
   default:
      return (FALSE);
   }

}
static BOOL is_value_rate (tiUINT32 rate);
struct CHAN_FREQ {
    UINT8       chan;
    UINT32      freq;
} ChanFreq[] = {
    {1,2412000}, {2,2417000}, {3,2422000}, {4,2427000},
    {5,2432000}, {6,2437000}, {7,2442000}, {8,2447000},
    {9,2452000},
    {10,2457000}, {11,2462000}, {12,2467000}, {13,2472000},
    {14,2484000}, {36,5180000}, {40,5200000}, {44,5220000},
    {48,5240000}, {52,5260000}, {56,5280000}, {60,5300000},
    {64,5320000},
    {100,5500000}, {104,5520000}, {108,5540000}, {112,5560000},
    {116,5580000}, {120,5600000}, {124,5620000}, {128,5640000},
    {132,5660000}, {136,5680000}, {140,5700000}, {149,5745000},
    {153,5765000}, {157,5785000}, {161,5805000} };

static named_value_t power_mode_val[] = {
        { OS_POWER_MODE_AUTO,        "AUTO" },
        { OS_POWER_MODE_ACTIVE,      "ACTIVE" },
        { OS_POWER_MODE_SHORT_DOZE,  "SHORT_DOZE" },
        { OS_POWER_MODE_LONG_DOZE,   "LONG_DOZE" }
};

static named_value_t power_level_val[] = {
        { OS_POWER_LEVEL_ELP,        "ELP" },
        { OS_POWER_LEVEL_PD,      "PD" },
        { OS_POWER_LEVEL_AWAKE,  "AWAKE" }
};

static named_value_t encrypt_type[] = {
        { OS_ENCRYPTION_TYPE_NONE,              "None" },
        { OS_ENCRYPTION_TYPE_WEP,               "WEP"  },
        { OS_ENCRYPTION_TYPE_TKIP,              "TKIP" },
        { OS_ENCRYPTION_TYPE_AES,               "AES" }
};

static named_value_t scanType2Str[] = {
        { SCAN_TYPE_NORMAL_PASSIVE,             "Passive Normal Scan"               },
        { SCAN_TYPE_NORMAL_ACTIVE,              "Active Normal Scan"                },
        { SCAN_TYPE_SPS,                        "Scheduled Passive Scan (SPS)"      },
        { SCAN_TYPE_TRIGGERED_PASSIVE,          "Passive Triggered Scan"            },
        { SCAN_TYPE_TRIGGERED_ACTIVE,           "Active Triggered Scan"             }
};

static named_value_t band2Str[] = {
        { RADIO_BAND_2_4_GHZ,                   "2.4 GHz"                        },
        { RADIO_BAND_5_0_GHZ,                   "5.0 GHz"                        },
        { RADIO_BAND_DUAL,                      "Both   "                        }
};

static named_value_t rate2Str[] = {
        { DRV_RATE_MASK_AUTO,                   "Auto    "                          },
        { DRV_RATE_MASK_1_BARKER,               "1 Mbps  "                          },
        { DRV_RATE_MASK_2_BARKER,               "2 Mbps  "                          },
        { DRV_RATE_MASK_5_5_CCK,                "5.5 Mbps"                          },
        { DRV_RATE_MASK_11_CCK,                 "11 Mbps "                          },
        { DRV_RATE_MASK_22_PBCC,                "22 Mbps "                          },
        { DRV_RATE_MASK_6_OFDM,                 "6 Mbps  "                          },
        { DRV_RATE_MASK_9_OFDM,                 "9 Mbps  "                          },
        { DRV_RATE_MASK_12_OFDM,                "12 Mbps "                          },
        { DRV_RATE_MASK_18_OFDM,                "18 Mbps "                          },
        { DRV_RATE_MASK_24_OFDM,                "24 Mbps "                          },
        { DRV_RATE_MASK_36_OFDM,                "36 Mbps "                          },
        { DRV_RATE_MASK_48_OFDM,                "48 Mbps "                          },
        { DRV_RATE_MASK_54_OFDM,                "54 Mbps "                          }
};

static named_value_t EtEvent2Str[] = {
        { SCAN_ET_COND_DISABLE,                      "ET disabled  "                     },
        { SCAN_ET_COND_BEACON,                       "ET on Beacon "                     },
        { SCAN_ET_COND_PROBE_RESP,                   "ET on Prb Rsp"                     },
        { SCAN_ET_COND_ANY_FRAME,                    "ET on both   "                     }
};

/* used in scan_display */
static char* rate2StrFunc(UINT32 rate)
{
    UINT32 i;

    for ( i = 0; i < RATE_TABLE_SIZE; i++ )
    {
        if ( rate2Str[ i ].value == rate )
            return rate2Str[ i ].name;
    }
    return rate2Str[ 0 ].name;
}

/* used in scan_display */
static char* EtEvent2StrFunc( UINT32 ETCond )
{
    int i;

    for ( i = 0; i < ET_TABLE_SIZE; i++ )
    {
        if ( EtEvent2Str[ i ].value == ETCond )
        {
            return EtEvent2Str[ i ].name;
        }
    }

    return EtEvent2Str[ 0 ].name;
}

/* used in get_bssid_list() */
UINT8 Freq2Chan(UINT32 freq)
{
    UINT32 i;

    for(i=0; i<CHAN_FREQ_TABLE_SIZE; i++)
        if(ChanFreq[i].freq == freq) return ChanFreq[i].chan;

    return 0;
}
/* IPC events Callback */
int cli_receive_ev(IPC_EV_DATA*    pData)
{
    tiUINT8 *buf;
    OS_802_11_QOS_TSPEC_PARAMS *AddTsResult;
    OS_802_11_AUTHENTICATION_REQUEST *request;
	OS_802_11_DISASSOCIATE_REASON_T	 *pDisAssoc;
    OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS *CrossParams;
    OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_CROSS_INDICATION_PARAMS *TrafficIntensityThresholdParams;
	btCoexStatus_t				*btCoexStatus;

#if defined (_WINDOWS)
#endif
    g_tester_receive_event((tiUINT8)((IPC_EVENT_PARAMS *)pData)->uEventType);
#if defined (_WINDOWS)
#endif

    switch(((IPC_EVENT_PARAMS *)pData)->uEventType)
    {
        case IPC_EVENT_ASSOCIATED:
            console_printf_terminal("CLI Event - Associated\n");
// TRS:HLC
/*
#if defined (_WINDOWS)
#endif
*/
//TRS end
        break;
        case IPC_EVENT_DISASSOCIATED:
			pDisAssoc = (OS_802_11_DISASSOCIATE_REASON_T*)pData->uBuffer;
			switch(pDisAssoc->eDisAssocType)
			{
			case OS_DISASSOC_STATUS_UNSPECIFIED:
				console_printf_terminal("CLI Event - Disassociated with unspecified reason (User/SG/Recovery)\n");
				break;
			case OS_DISASSOC_STATUS_AUTH_REJECT:
				if (pDisAssoc->uStatusCode == STATUS_PACKET_REJ_TIMEOUT)
				{
					console_printf_terminal("CLI Event - Disassociated due to no Auth response \n");
				}
				else
				{
					console_printf_terminal("CLI Event - Disassociated due to Auth response packet with reason = %d\n", pDisAssoc->uStatusCode);
				}
				break;
			case OS_DISASSOC_STATUS_ASSOC_REJECT:
				if (pDisAssoc->uStatusCode == STATUS_PACKET_REJ_TIMEOUT)
				{
					console_printf_terminal("CLI Event - Disassociated due to no Assoc response \n");
				}
				else
				{
					console_printf_terminal("CLI Event - Disassociated due to Assoc response packet with reason = %d\n", pDisAssoc->uStatusCode);
				}
			    break;
			case OS_DISASSOC_STATUS_SECURITY_FAILURE:
				console_printf_terminal("CLI Event - Disassociated due to RSN failure\n");
			    break;
			case OS_DISASSOC_STATUS_AP_DEAUTHENTICATE:
				console_printf_terminal("CLI Event - Disassociated due to AP deAuthenticate packet with reason = %d\n", pDisAssoc->uStatusCode);
				break;
			case OS_DISASSOC_STATUS_AP_DISASSOCIATE:
				console_printf_terminal("CLI Event - Disassociated due to AP disAssoc packet with reason = %d\n", pDisAssoc->uStatusCode);
				break;
			case OS_DISASSOC_STATUS_ROAMING_TRIGGER:
				console_printf_terminal("CLI Event - Disassociated due to roaming trigger = %d\n", pDisAssoc->uStatusCode);
				break;
			default:
				console_printf_terminal("CLI Event - Disassociated with unknown reason = %d\n", pDisAssoc->eDisAssocType);
			    break;
			}
        break;
        case IPC_EVENT_LINK_SPEED:
            console_printf_terminal("CLI Event - LinkSpeed\n");
        break;
        case IPC_EVENT_AUTH_SUCC:
            console_printf_terminal("CLI Event - Authentication Success\n");
        break;
        case IPC_EVENT_SCAN_COMPLETE:
            console_printf_terminal("CLI Event - Scan Complete\n");
        break;
        case IPC_EVENT_TIMEOUT:
            console_printf_terminal("CLI Event - Timeout\n");
        break;
        case IPC_EVENT_UNBOUND:
            console_printf_terminal("CLI Event - Unbound\n");
        break;
        case IPC_EVENT_BOUND:
            console_printf_terminal("CLI Event - Bound\n");
        break;
        case IPC_EVENT_EAPOL:
            console_printf_terminal("CLI Event - EAPOL\n");
        break;
        case IPC_EVENT_MEDIA_SPECIFIC:
            buf = pData->uBuffer;
            request = (OS_802_11_AUTHENTICATION_REQUEST *) (buf + sizeof(tiUINT32));
            if( request->Flags == OS_802_11_REQUEST_PAIRWISE_ERROR ||
                request->Flags == OS_802_11_REQUEST_GROUP_ERROR)
            console_printf_terminal("CLI Event - Media_Specific\n");
        break;
        case IPC_EVENT_CCKM_START:
            console_printf_terminal("CLI Event - CCKM_Start\n");
        break;

        case IPC_EVENT_PREAUTH_EAPOL:
            console_printf_terminal("CLI Event - PreAuth EAPOL\n");
            break;

        case IPC_EVENT_LOW_SNR:
            console_printf_terminal("CLI Event - Low SNR\n");
            break;

        case IPC_EVENT_LOW_RSSI:
            console_printf_terminal("CLI Event - Low RSSI\n");
            break;

        case IPC_EVENT_EAP_AUTH_FAILURE:
            {
                OS_802_11_EAP_TYPES eapType;
                tiINT32             status;
                authStatus_e        eAuthStatusReason;

				/* Cast the value to be UINT16 16 bits since it is sent this way in the EXCMngr.c */
                eAuthStatusReason = (authStatus_e)((*(pData->uBuffer)) & 0xFFFF);

                status = TI_GetEAPType( g_id_adapter, &eapType );

                if (eapType == OS_EAP_TYPE_LEAP)
                {
                    console_printf_terminal("CLI Event - LEAP Authentication Failed \n");
                }
                else if (eapType == OS_EAP_TYPE_FAST)
                {
                    console_printf_terminal("CLI Event - EAP-FAST Authentication Failed \n");
                }
                else if (eapType == OS_EAP_TYPE_TLS)
                {
                    console_printf_terminal("CLI Event - EAP-TLS Authentication Failed \n");
                }

                switch ( eAuthStatusReason )
                {
                case RSN_AUTH_STATUS_INVALID_TYPE:
                    console_printf_terminal("The reason: Invalid Authentication Type \n");
                    break;
                case RSN_AUTH_STATUS_TIMEOUT:
                    console_printf_terminal("The reason: Authentication Timeout \n");
                    break;
                case RSN_AUTH_STATUS_CHALLENGE_FROM_AP_FAILED:
                case RSN_AUTH_STATUS_CHALLENGE_TO_AP_FAILED:
                    console_printf_terminal("The reason: username or password incorrect \n");
                    break;
                default:
                    console_printf_terminal("The reason: unknown = %d \n", eAuthStatusReason);
                    break;
                }

            }
            break;

        case IPC_EVENT_TSPEC_STATUS:
           AddTsResult = (OS_802_11_QOS_TSPEC_PARAMS *)pData->uBuffer;
           console_printf_terminal("CLI Event - IPC_EVENT_TSPEC_STATUS -- (userPriority = %d, ReasonCode = %d) \n",AddTsResult->uUserPriority,AddTsResult->uReasonCode);
           console_printf_terminal ("Tspec Parameters (as received through event handler):\n");
           console_printf_terminal ("-----------------------------------------------------\n");
           console_printf_terminal ("userPriority = %d\n",AddTsResult->uUserPriority);
           console_printf_terminal ("uNominalMSDUsize = %d\n",AddTsResult->uNominalMSDUsize);
           console_printf_terminal ("uMeanDataRate = %d\n",AddTsResult->uMeanDataRate);
           console_printf_terminal ("uMinimumPHYRate = %d\n",AddTsResult->uMinimumPHYRate);
           console_printf_terminal ("uSurplusBandwidthAllowance = %d\n",AddTsResult->uSurplusBandwidthAllowance);
           console_printf_terminal ("uAPSDFlag = %d\n",AddTsResult->uAPSDFlag);
           console_printf_terminal ("uMediumTime = %d\n\n",AddTsResult->uMediumTime);
           break;
        case IPC_EVENT_TSPEC_RATE_STATUS:
           CrossParams = (OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS *)pData->uBuffer;
           console_printf_terminal("CLI Event - IPC_EVENT_TSPEC_RATE_STATUS (AC = %d, HighLowFlag = %d, AboveOrBelow = %d)\n",CrossParams->uAC,CrossParams->uHighOrLowThresholdFlag,CrossParams->uAboveOrBelowFlag);
           break;
        case IPC_EVENT_MEDIUM_TIME_CROSS:
           CrossParams = (OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS *)pData->uBuffer;
           console_printf_terminal("CLI Event - IPC_EVENT_MEDIUM_TIME_CROSS (AC = %d, HighLowFlag = %d, AboveOrBelow = %d)\n",CrossParams->uAC,CrossParams->uHighOrLowThresholdFlag,CrossParams->uAboveOrBelowFlag);
           break;
        case IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED:
           TrafficIntensityThresholdParams = (OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_CROSS_INDICATION_PARAMS*)pData->uBuffer;
           printf("CLI Event - IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED\n");
           printf("---------------------------------------------------------\n");
           printf("Threshold crossed: %s Threshold\n", ((TrafficIntensityThresholdParams->uHighOrLowThresholdFlag == HIGH_THRESHOLD_CROSS) ? "High" : "Low"));
           printf("Direction crossed: %s\n\n", ((TrafficIntensityThresholdParams->uAboveOrBelowFlag == CROSS_ABOVE) ? "Above" : "Below"));
           break;

        case IPC_EVENT_WPA2_PREAUTHENTICATION:
            {
                ULONG *WPA2_PreAuth_Status;

               WPA2_PreAuth_Status = (ULONG *)pData->uBuffer;
               printf("CLI Event - IPC_EVENT_WPA2_PREAUTHENTICATION\n");
               printf("Status code = %lu\n",*WPA2_PreAuth_Status);
            }
           break;

        case IPC_EVENT_ROAMING_COMPLETE:
            printf("CLI Event - IPC_EVENT_ROAMING_COMPLETE \n");
        break;

	case IPC_EVENT_BT_COEX_MODE:
		btCoexStatus = (btCoexStatus_t*)pData->uBuffer;
		if (btCoexStatus->state)
		{
			console_printf_terminal("CLI Event - IPC_EVENT_BT_COEX_MODE (SG is ON, minTxRate = %d)\n",btCoexStatus->minTxRate);
		}
		else
		{
			console_printf_terminal("CLI Event - IPC_EVENT_BT_COEX_MODE (SG is OFF)\n");
		}
		break;

        default:
            console_printf_terminal("CLI Event - Unknown event\n");
        break;
    }
    return 0;


}

static char *print_rate(rate_e rate)
{
    static char buf[20];

    if( rate == 0 )
        return "Auto (0)";

    sprintf(buf, "%.3g Mbps (%u%s)", RATE_2_MBPS(rate), rate,
        IS_BASIC_RATE(rate) ? " - basic" : "" );
    return buf;
}

static BOOL isJunkSSID(OS_802_11_SSID *ssid )
{
    if ((ssid == NULL) || (ssid->SsidLength == 0))
    {
        return TRUE;
    }

    if (ssid->SsidLength > 2)
    {
        if ((ssid->Ssid[0] < MAX_SSID_LEN) &&
            (ssid->Ssid[1] < MAX_SSID_LEN) &&
            (ssid->Ssid[2] < MAX_SSID_LEN))
        {
            return TRUE;
        }
    }

    return FALSE;
}

static char *get_ssid_string(OS_802_11_SSID *ssid )
{
    static char tmp_buf[MAX_SSID_LEN+1];
    if ((ssid == NULL) || (ssid->SsidLength == 0) || isJunkSSID(ssid))
        return "<empty>";

    if (ssid->SsidLength < SIZE_ARR(ssid->Ssid))
           ssid->Ssid[ssid->SsidLength] = 0;
    else if( ssid->SsidLength == MAX_SSID_LEN )
    {
        memcpy(tmp_buf, ssid->Ssid, MAX_SSID_LEN );
        tmp_buf[MAX_SSID_LEN] = 0;
        return tmp_buf;
    }
    else
    {
        console_printf_terminal("error: invalid ssid length (len = %u)\n", ssid->SsidLength );
        return "<error>";
    }
    return (char *) ssid->Ssid;
}
/*  return 0 on error
    or 'mac' if success
    'str' format: num.num.num.num.num.num (where 'num' is number from 0 to 255)
*/
U8* str2MACAddr(char *str, U8 *mac)
{
    const int mac_len = 6;
    int i;
    char *p = str;
#ifndef _WINDOWS
    errno = 0;
#endif
    console_printf_terminal("str2MAC():");
    for( i=0; i<mac_len; i++ )
    {
        mac[i] = (U8) strtoul(p, &p, 16);
#ifndef _WINDOWS
        if( errno == ERANGE )
            return NULL;
#endif
        console_printf_terminal("%2x.", mac[i] );
        p++;

    }
    console_printf_terminal("\n");
    return mac;
}

static char * print_mac_2_str(OS_802_11_MAC_ADDRESS mac)
{
    static char buf[30];

    sprintf(buf, "%02x.%02x.%02x.%02x.%02x.%02x",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return buf;
}

/* convert hex MAC addr in the form xx:xx:xx:xx:xx:xx to mac address */
static void hexStr2MACAddr( char *hexStr, macAddress_t* mac )
{
    size_t i;
    char *currHexStrPos = hexStr;
    UINT8 tempValue;

    /* convert hex string to lower */
    for ( i = 0; i < strlen(hexStr); i++ )
    {
        hexStr[ i ] = tolower( hexStr[ i ] );
    }

    /* convert to numbers */
    for ( i = 0; i < 6; i++ )
    {
        tempValue = 0;
        if ( *currHexStrPos >= 'a' )
            tempValue = *currHexStrPos - 'a' + 10;
        else
            tempValue = *currHexStrPos - '0';
        currHexStrPos++;
        tempValue <<= 4;
        if ( *currHexStrPos >= 'a' )
            tempValue += *currHexStrPos - 'a' + 10;
        else
            tempValue += *currHexStrPos - '0';
        currHexStrPos += 2;
        mac->addr[ i ] = tempValue;
    }
}

static void mac2HexStr( macAddress_t* mac, char* hexStr )
{
    sprintf( hexStr, "%02x:%02x:%02x:%02x:%02x:%02x",
        mac->addr[0],mac->addr[1],mac->addr[2],mac->addr[3],mac->addr[4],mac->addr[5]);
}

char *get_name_by_value(named_value_t *arr, int arr_size, UINT32 value)
{
    int i;
    for(i=0; i<arr_size; i++) {
        if( arr[i].value == value )
            return arr[i].name;
    }
    return "<unknow>";
}

void cmd_modify_supported_rates(ConParm_t parm[], U16 nParms)
{
    rates_t data = { 0 };
    char debug_buf[256] = { 0 };
    int i;

    if( nParms == 0 )
    {
        if( !TI_GetSupportedRates(g_id_adapter, (tiUINT8*)&data, sizeof(data)) )
        {
            console_printf_terminal(" Rates: ");
            for( i=0; i<MAX_SUPPORTED_RATES && i < data.len; i++ )
            {
                console_printf_terminal("%g Mbps(%u%s)%s", RATE_2_MBPS(data.ratesString[i]),
                    data.ratesString[i],
                    IS_BASIC_RATE(data.ratesString[i]) ? " - basic" : "",
                    (i < data.len-1) ? "," : "" );
            }
            console_printf_terminal("\n");
        }
    }
    else
    {
        char *end_p, *buf = (char *) parm[0].value;
        float val;
        console_printf_terminal("param: %s\n", buf );
#ifndef _WINDOWS
        errno = 0;
#endif
        for( i=0; *buf && i < MAX_SUPPORTED_RATES; i++ )
        {
            val = (float) strtod(buf, &end_p);
            if(
#ifndef _WINDOWS
			   errno ||
#endif
			   buf == end_p )
            {
                console_printf_terminal("cmd_modify_supported_rates(): invalid value - %s\n", buf );
                return;
            }
            if( val > ((1 << (sizeof(data.ratesString[i]) * 8))-1) )
            {
                console_printf_terminal("cmd_modify_supported_rates(): out of range '%.*s'\n", end_p - buf, buf );
                return;
            }
            data.ratesString[i] = (tiUINT8) (val);

            sprintf(&debug_buf[strlen(debug_buf)], "%g (%d) ", val, data.ratesString[i] );

            buf = end_p;
            while( *buf==' ' || *buf == ',' )   buf++;

        }

        if( *buf )
        {
            console_printf_terminal("too many parameters. Max=%d\n", MAX_SUPPORTED_RATES );
            return;
        }

        data.len = i;
        console_printf_terminal("**set rates (%d) :%s\n", data.len, debug_buf );
        TI_SetSupportedRates(g_id_adapter, (tiUINT8*) &data, sizeof(data));
    }
}

void cmd_show_status(ConParm_t parm[], U16 nParms)
{
    tiINT32 res;
    OS_802_11_BSSID_EX BssIdInfo;

    tiUINT32 data;
    char buf[4096] = { 0 };
    OS_802_11_SSID ssid = { 0 };
    OS_802_11_MAC_ADDRESS bssid = { 0 };

    UNUSED(parm);
    UNUSED(nParms);

    buf[0] = 0;

    res = TI_WLAN_IsDriverRun(g_id_adapter, (tiBOOL *)&data);
    if( res )
        return ;

    sprintf(buf, "Status   : %s\n", res ? "<error>" : (data ? "running" : "stopped") );

    sprintf(&buf[strlen(buf)], "MAC      : ");
    if( !TI_GetCurrentAddress(g_id_adapter, &bssid ) )
        strcat(buf, print_mac_2_str(bssid) );
    else
        strcat(buf, "<error>");

    strcat(buf, "\nSSID     : ");
    if( !TI_GetCurrentSSID(g_id_adapter, &ssid) )
    {
        if (!isJunkSSID(&ssid))
        {
           strncat(buf, get_ssid_string(&ssid),ssid.SsidLength);
        }
        else
        {
           strcat(buf,"<empty>");
        }
    }
    else
        strcat(buf, "<error>" );

    sprintf(&buf[strlen(buf)], "\nBSSID    : ");

   TI_GetSelectedBSSIDInfo(g_id_adapter, &BssIdInfo);
   strcat(buf, print_mac_2_str(BssIdInfo.MacAddress));

    strcat(buf, "\nChannel  : ");
    data = 0;
    if( !TI_GetCurrentChannel(g_id_adapter, &data ) )
    {
        sprintf(&buf[strlen(buf)], "%d", data );
    }
    else
        strcat(buf, "<error>");

    console_printf_terminal("==========================\n%s\n==========================\n\n", buf);
}

void cmd_connect(ConParm_t parm[], U16 nParms)
{
    char buf[1] = { 0 };
    OS_802_11_MAC_ADDRESS bssid = { 0xff,0xff,0xff,0xff,0xff,0xff };

    buf[0] = 0;

    switch (nParms) {

    case 0 :
        /*
         *  No SSID & No BSSID are set -
         *  Use Any SSID & Any BSSID.
         */

        TI_SetBSSID(g_id_adapter, &bssid );
        TI_SetSSID(g_id_adapter, (tiUINT8 *) buf);
        break;

    case 1:
        /*
         *  SSID set & BSSID insn't set  -
         *  Use CLI's SSID & Any BSSID.
         */
        TI_SetBSSID(g_id_adapter, &bssid );
        TI_SetSSID(g_id_adapter, (tiUINT8 *) parm[0].value);
        break;

    case 2:
        /*
         *  Both SSID & BSSID are set -
         *  Use CLI's SSID & BSSID.
         */
        if( !str2MACAddr((char *) parm[1].value, bssid) )
            return;
        TI_SetBSSID(g_id_adapter, &bssid);
        TI_SetSSID(g_id_adapter, (tiUINT8 *) parm[0].value);
        break;
}
}


void cmd_disassociate(ConParm_t parm[], U16 nParms)
{
    UNUSED(parm);
    UNUSED(nParms);

    TI_Disassociate(g_id_adapter);
}

void cmd_show_advanced_params(ConParm_t parm[], U16 nParms)
{
    tiUINT32 mode_4x = 0, preamble = 0xfefefefe;
    OS_802_11_AUTHENTICATION_MODE auth;
    OS_802_11_ENCRYPTION_TYPES encrypt;
    tiUINT32    rts_tresh;
    tiUINT32    frag_tresh = 0xfefefefe;
    tiUINT8     tx_power = 0xfe;
    OS_802_11_POWER_PROFILE power_mode = 0;

    UNUSED(parm);
    UNUSED(nParms);

    TI_GetAuthenticationMode(g_id_adapter, &auth );
    TI_GetTxPowerDbm(g_id_adapter, (tiCHAR*) &tx_power);
    TI_GetFragmentThreshold( g_id_adapter, &frag_tresh);
    TI_GetRTSThreshold( g_id_adapter, &rts_tresh);
    TI_Get4XState( g_id_adapter, &mode_4x);
    TI_GetEncryptionType( g_id_adapter, &encrypt );
    TI_GetPowerMode(g_id_adapter, (OS_802_11_POWER_PROFILE*) &power_mode );
    console_printf_terminal("  Authentication : %u\n", auth );
    console_printf_terminal("      Power mode : %d\n", (tiUINT32) power_mode );
    console_printf_terminal("  Tx Power (Dbm/10 units) : %d\n", tx_power );

    console_printf_terminal("      Encryption : %u\n", encrypt );
    console_printf_terminal("        Preamble : <%s>\n", (preamble) ? "short" : "long");
/*    console_printf_terminal("      Tx antenna : %u\n", tx_ant );*/
/*    console_printf_terminal(  "      Rx antenna : %u\n", rx_ant );*/
/*    console_printf_terminal(  "Ap Tx power level: n/a\n" );*/
/*    console_printf_terminal(  "  Tx power value : %lu\n", tx_power_val );*/
/*    console_printf_terminal(  "     MAC address : n/a\n" );*/
    console_printf_terminal(  "        4X state : %d\n", mode_4x );
/*     console_printf_terminal(  "        B/G mode : n/a\n" );*/
/*     console_printf_terminal(  "    Country code : n/a\n" );*/
/*     console_printf_terminal(  "      IP address : n/a\n" );*/
/*     console_printf_terminal(  "        Net mask : n/a\n" );*/
/*     console_printf_terminal(  "         Gateway : n/a\n" );*/
    console_printf_terminal(  " Frag. threshold : %u\n", frag_tresh);
    console_printf_terminal(  "   RTS threshold : %u\n", rts_tresh );

    console_printf_terminal(  "Power mode: ");
    print_available_values(power_mode_val);

    console_printf_terminal(  "Encryption type: ");
    print_available_values(encrypt_type);
}

void cmd_show_statistics(ConParm_t parm[], U16 nParms)
{
    TIWLN_STATISTICS statistics = { 0 } ;

    UNUSED(parm);
    UNUSED(nParms);

    console_printf_terminal(  "TI_GetStatistics(%s, data=%p, size=%d\n", (char*) g_id_adapter, &statistics, sizeof(statistics) );

    if( TI_GetStatistics( g_id_adapter, &statistics ) )
    {
        console_printf_terminal ("Error in getting TI_GetStatistics!!\n");
        return;
    }

    console_printf_terminal("******************\n");
    console_printf_terminal("Driver Statistics:\n");
    console_printf_terminal("******************\n");

    console_printf_terminal("    dot11CurrentTxRate : %s\n", print_rate(statistics.dot11CurrentTxRate) );
    console_printf_terminal("   dot11CurrentChannel : %d\n", statistics.dot11CurrentChannel );
    console_printf_terminal("     currentMACAddress : %s\n", print_mac_2_str(statistics.currentMACAddress) );
    console_printf_terminal("      dot11DesiredSSID : %s\n", get_ssid_string(&statistics.dot11DesiredSSID) );
    console_printf_terminal("          dot11BSSType : %d\n", statistics.dot11BSSType );
    console_printf_terminal("    AuthenticationMode : %d\n", statistics.AuthenticationMode );
    console_printf_terminal("    bShortPreambleUsed : %d\n", statistics.bShortPreambleUsed );
    console_printf_terminal("          RTSThreshold : %d\n", statistics.RTSThreshold );
    console_printf_terminal("FragmentationThreshold : %d\n", statistics.FragmentationThreshold );
    console_printf_terminal(" bDefaultWEPKeyDefined : %d\n", statistics.bDefaultWEPKeyDefined );
    console_printf_terminal("             WEPStatus : %d\n", statistics.WEPStatus );
    console_printf_terminal("             TxAntenna : %d\n", statistics.TxAntenna );
    console_printf_terminal("             RxAntenna : %d\n", statistics.RxAntenna );
    console_printf_terminal("            TxPowerDbm : %d\n", statistics.TxPowerDbm );
    console_printf_terminal("             PowerMode : %d\n", statistics.PowerMode );
    console_printf_terminal("               RxLevel : %d\n", statistics.RxLevel );

    /**/
    /* status & AP info*/
    /**/
    console_printf_terminal("            dot11State : %d\n", statistics.dot11State );

    /**/
    /* network layer statistics*/
    /**/
    console_printf_terminal("                RecvOk : %d\n", statistics.tiCounters.RecvOk );
    console_printf_terminal("             RecvError : %d\n", statistics.tiCounters.RecvError );
    console_printf_terminal("     DirectedBytesRecv : %d\n", statistics.tiCounters.DirectedBytesRecv );
    console_printf_terminal("    DirectedFramesRecv : %d\n", statistics.tiCounters.DirectedFramesRecv );
    console_printf_terminal("    MulticastBytesRecv : %d\n", statistics.tiCounters.MulticastBytesRecv );
    console_printf_terminal("   MulticastFramesRecv : %d\n", statistics.tiCounters.MulticastFramesRecv );
    console_printf_terminal("    BroadcastBytesRecv : %d\n", statistics.tiCounters.BroadcastBytesRecv );
    console_printf_terminal("   BroadcastFramesRecv : %d\n", statistics.tiCounters.BroadcastFramesRecv );
    console_printf_terminal("             FcsErrors : %d\n", statistics.tiCounters.FcsErrors );
    console_printf_terminal("           BeaconsRecv : %d\n", statistics.tiCounters.BeaconsRecv );
    console_printf_terminal("          AssocRejects : %d\n", statistics.tiCounters.AssocRejects );
    console_printf_terminal("         AssocTimeouts : %d\n", statistics.tiCounters.AssocTimeouts );
    console_printf_terminal("           AuthRejects : %d\n", statistics.tiCounters.AuthRejects );
    console_printf_terminal("          AuthTimeouts : %d\n", statistics.tiCounters.AuthTimeouts );

    /**/
    /* other statistics*/
    /**/
    console_printf_terminal("        dwSecuritySuit : %d\n", statistics.dwSecuritySuit );
    console_printf_terminal("       dwSecurityState : %d\n", statistics.dwSecurityState );
    console_printf_terminal("  dwSecurityAuthStatus : %d\n", statistics.dwSecurityAuthStatus );
    console_printf_terminal("         dwFeatureSuit : %d\n", statistics.dwFeatureSuit );
}

void cmd_show_tx_statistics(ConParm_t parm[], U16 nParms)
{
    TIWLN_TX_STATISTICS statistics;
    UINT32 TxQid;
    UINT32 AverageDelay;
    UINT32 AverageFWDelay;
    UINT32 AverageMacDelay;
    tiINT32 status;

    UNUSED(parm);
    UNUSED(nParms);

    console_printf_terminal("TI_GetTxStatistics(%s, data=%p, size=%d\n", (char*) g_id_adapter, &statistics, sizeof(statistics) );

    /* The first parameter indicates whether to clear the statistics on read: 0 - don't clear, 1 - clear */
    if ( 0 == nParms )
    {
        status = TI_GetTxStatistics( g_id_adapter, &statistics, 0 );
    }
    else
    {
        status = TI_GetTxStatistics( g_id_adapter, &statistics, parm[0].value );
    }

    if( status )
    {
        console_printf_terminal ("Error in getting TI_GetTxStatistics!!\n");
        return;
    }

    console_printf_terminal("*********************\n");
    console_printf_terminal("Tx Queues Statistics:\n");
    console_printf_terminal("*********************\n");

    for (TxQid = 0; TxQid < MAX_NUM_OF_TX_QUEUES; TxQid++)
    {
        console_printf_terminal("\nTx Queue %d:\n", TxQid);
        console_printf_terminal("===========\n");

        console_printf_terminal("  Total Good Frames             : %d\n", statistics.txCounters[TxQid].XmitOk );
        console_printf_terminal("  Unicast Bytes                 : %d\n", statistics.txCounters[TxQid].DirectedBytesXmit );
        console_printf_terminal("  Unicast Frames                : %d\n", statistics.txCounters[TxQid].DirectedFramesXmit );
        console_printf_terminal("  Multicast Bytes               : %d\n", statistics.txCounters[TxQid].MulticastBytesXmit );
        console_printf_terminal("  Multicast Frames              : %d\n", statistics.txCounters[TxQid].MulticastFramesXmit );
        console_printf_terminal("  Broadcast Bytes               : %d\n", statistics.txCounters[TxQid].BroadcastBytesXmit );
        console_printf_terminal("  Broadcast Frames              : %d\n", statistics.txCounters[TxQid].BroadcastFramesXmit );
        console_printf_terminal("  Retry Failures                : %d\n", statistics.txCounters[TxQid].RetryFailCounter );
        console_printf_terminal("  Tx Timeout Failures           : %d\n", statistics.txCounters[TxQid].TxTimeoutCounter );
        console_printf_terminal("  No Link Failures              : %d\n", statistics.txCounters[TxQid].NoLinkCounter );
        console_printf_terminal("  Other Failures                : %d\n", statistics.txCounters[TxQid].OtherFailCounter );
        console_printf_terminal("  Max Consecutive Retry Failures : %d\n\n", statistics.txCounters[TxQid].MaxConsecutiveRetryFail );

        console_printf_terminal("  Retry histogram:\n");
        console_printf_terminal("  ----------------\n\n");
        console_printf_terminal("  Retries: %8d %8d %8d %8d %8d %8d %8d %8d\n", 0, 1, 2, 3, 4, 5, 6, 7);
        console_printf_terminal("  packets: %8d %8d %8d %8d %8d %8d %8d %8d\n\n",
                                statistics.txCounters[TxQid].RetryHistogram[ 0 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 1 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 2 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 3 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 4 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 5 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 6 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 7 ]);
        console_printf_terminal("  Retries: %8d %8d %8d %8d %8d %8d %8d %8d\n", 8, 9, 10, 11, 12, 13, 14, 15);
        console_printf_terminal("  packets: %8d %8d %8d %8d %8d %8d %8d %8d\n\n",
                                statistics.txCounters[TxQid].RetryHistogram[ 8 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 9 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 10 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 11 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 12 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 13 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 14 ],
                                statistics.txCounters[TxQid].RetryHistogram[ 15 ]);

        if (statistics.txCounters[TxQid].NumPackets)
        {
            AverageDelay = statistics.txCounters[TxQid].SumTotalDelayMs / statistics.txCounters[TxQid].NumPackets;
            AverageFWDelay = statistics.txCounters[TxQid].SumFWDelayUs / statistics.txCounters[TxQid].NumPackets;
            AverageMacDelay = statistics.txCounters[TxQid].SumMacDelayUs / statistics.txCounters[TxQid].NumPackets;
        }
        else
        {
            AverageDelay = 0;
            AverageFWDelay = 0;
            AverageMacDelay = 0;
        }

        console_printf_terminal("  Total Delay ms (average/sum) : %d / %d\n", AverageDelay, statistics.txCounters[TxQid].SumTotalDelayMs);
        console_printf_terminal("  FW Delay us (average/sum) : %d / %d\n", AverageFWDelay, statistics.txCounters[TxQid].SumFWDelayUs);
        console_printf_terminal("  MAC Delay us (average/sum)   : %d / %d\n\n", AverageMacDelay, statistics.txCounters[TxQid].SumMacDelayUs);

        console_printf_terminal("  Delay Ranges [msec]  : Num of packets\n");
        console_printf_terminal("  -------------------  : --------------\n");
        console_printf_terminal("        0   -    1     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_0_TO_1] );
        console_printf_terminal("        1   -   10     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_1_TO_10] );
        console_printf_terminal("       10   -   20     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_10_TO_20] );
        console_printf_terminal("       20   -   40     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_20_TO_40] );
        console_printf_terminal("       40   -   60     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_40_TO_60] );
        console_printf_terminal("       60   -   80     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_60_TO_80] );
        console_printf_terminal("       80   -  100     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_80_TO_100] );
        console_printf_terminal("      100   -  200     : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_100_TO_200] );
        console_printf_terminal("        Above 200      : %d\n", statistics.txCounters[TxQid].txDelayHistogram[TX_DELAY_RANGE_ABOVE_200] );
    }
}

void cmd_show_about(ConParm_t parm[], U16 nParms)
{
    TIWLN_VERSION_EX data;

    UNUSED(parm);
    UNUSED(nParms);

    console_printf_terminal("Utility version: %s (%u.%u.%u)\n", SW_VERSION_STR, SW_RELEASE_DAY,
            SW_RELEASE_MONTH, SW_RELEASE_YEAR );
    if( TI_GetDriverVersion(g_id_adapter, &data ) )
        return ;

    console_printf_terminal("Driver version: %u.%u.%u.%u.%u\n", data.DrvVersion.major, data.DrvVersion.minor,
            data.DrvVersion.bugfix, data.DrvVersion.subld, data.DrvVersion.build );
    console_printf_terminal("Firmware version: %u.%u.%u.%u.%u\n", data.FWVersion.major, data.FWVersion.minor,
            data.FWVersion.bugfix, data.FWVersion.subld, data.FWVersion.build );
    console_printf_terminal("Eeprom Version: %u.%u.%u.%u.%u\n", data.HWVersion.major, data.HWVersion.minor,
            data.HWVersion.bugfix, data.HWVersion.subld, data.HWVersion.build );
    console_printf_terminal("Eeprom Version2: %u.%u.%u.%u.%u\n", data.NVVersion.major, data.NVVersion.minor,
            data.NVVersion.bugfix, data.NVVersion.subld, data.HWVersion.build );
}

void cmd_modify_ssid(ConParm_t parm[], U16 nParms)
{
    OS_802_11_SSID ssid = { 0 };
    OS_802_11_MAC_ADDRESS bssid = { 0xff,0xff,0xff,0xff,0xff,0xff };
    char *ssid_str;

    if( nParms == 0 )
    {
        if(!TI_GetCurrentSSID(g_id_adapter, &ssid))
            ssid_str = get_ssid_string(&ssid);
        else
            ssid_str = "<error>";
        console_printf_terminal("SSID: %s\n",  ssid_str );
    }
    else{
        /* Setting the new SSID, BRCS BSSID is set to clean pre-set BSSID */
        TI_SetBSSID(g_id_adapter, &bssid );
        TI_SetSSID(g_id_adapter, (tiUINT8 *) parm[0].value);
}
}

void cmd_modify_channel(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data1 = 0, data = 0;
        if( !TI_GetDesiredChannel( g_id_adapter, &data1) )
            console_printf_terminal("Channel=%d (desired: %d)\n", !TI_GetCurrentChannel(g_id_adapter, &data) ? data : -1, data1 );
    }
    else
        TI_SetDesiredChannel( g_id_adapter, parm[0].value );
}

void cmd_set_power_mode(ConParm_t parm[], U16 nParms)
{
    OS_802_11_POWER_PROFILE mode = 0x12345678;
    if( nParms )
        TI_ConfigPowerManagement (g_id_adapter, parm[0].value );
    else
    {
        if( !TI_GetPowerMode(g_id_adapter, &mode ) )
        {
            console_printf_terminal("Power mode: %d\n", mode );
            print_available_values(power_mode_val);
        }
    }
}

void cmd_set_PowerSave_PowerLevel(ConParm_t parm[], U16 nParms)
{
    OS_802_11_POWER_LEVELS mode;
    if( nParms )
        TI_SetPowerLevelPS (g_id_adapter, parm[0].value );
    else
    {
        if( !TI_GetPowerLevelPS(g_id_adapter, &mode ) )
        {
            console_printf_terminal("Power Level PowerSave: %d\n", mode );
            print_available_values(power_level_val);
        }
    }
}

void cmd_set_Default_PowerLevel(ConParm_t parm[], U16 nParms)
{
    OS_802_11_POWER_LEVELS mode;
    if( nParms )
        TI_SetPowerLevelDefault (g_id_adapter, parm[0].value );
    else
    {
        if( !TI_GetPowerLevelDefault(g_id_adapter, &mode ) )
        {
            console_printf_terminal("Default Power Level: %d\n", mode );
            print_available_values(power_level_val);
        }
    }
}

void cmd_set_DozeModeInAutoPowerLevel(ConParm_t parm[], U16 nParms)
{
	OS_802_11_POWER_PROFILE mode;
    if( nParms )
        TI_SetPowerLevelDozeMode (g_id_adapter, parm[0].value );
    else
    {
		/* set Short or Long Doze. no use of other parameters */
        if( !TI_GetPowerLevelDozeMode(g_id_adapter,&mode ) )
        {
            console_printf_terminal("Doze Mode in Auto Power Level: SHORT_DOZE -  %d LONG_DOZE - %d\n",
				OS_POWER_MODE_SHORT_DOZE,OS_POWER_MODE_LONG_DOZE);
        }
    }
}


void cmd_Beacon_Filter_Set_Desired_State(ConParm_t parm[], U16 nParms)
{
    /*there are two modes : feature ACTIVE & PASSIV ( or NOT ACTIVE )*/
    if( nParms )
    {
        TI_SetBeaconFilterDesiredState(g_id_adapter, parm[0].value );
    }
    else
    {
        console_printf_terminal("Use : 0 =  INACTIVE , 1 = ACTIVE\n" ) ;

    }
}

void cmd_Beacon_Filter_Get_Desired_State(ConParm_t parm[], U16 nParms)
{
    UINT8 desState = FALSE ;

    TI_GetBeaconFilterDesiredState(g_id_adapter, &desState ) ;
    console_printf_terminal("Desired State is %s\n", (desState == FALSE)?"FILTER INACTIVE":"FILTER ACTIVE" );


}



/* scan commands (new from eSTAdk 5.0) */

void init_scan_params(void)
{
    int i,j;

    /* init application scan default params */
    appScanParams.desiredSsid.len = 0;
    appScanParams.scanType = SCAN_TYPE_NORMAL_ACTIVE;
    appScanParams.band = RADIO_BAND_2_4_GHZ;
    appScanParams.probeReqNumber = 3;
    appScanParams.probeRequestRate = DRV_RATE_MASK_2_BARKER;
    appScanParams.numOfChannels = 11;
    for ( i = 0; i < 11; i++ )
    {
        for ( j = 0; j < 6; j++ )
        {
            appScanParams.channelEntry[ i ].normalChannelEntry.bssId.addr[ j ] = 0xff;
        }
        appScanParams.channelEntry[ i ].normalChannelEntry.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
        appScanParams.channelEntry[ i ].normalChannelEntry.ETMaxNumOfAPframes = 0;
        appScanParams.channelEntry[ i ].normalChannelEntry.maxChannelDwellTime = 60000;
        appScanParams.channelEntry[ i ].normalChannelEntry.minChannelDwellTime = 30000;
        appScanParams.channelEntry[ i ].normalChannelEntry.txPowerDbm = MAX_TX_POWER;
        appScanParams.channelEntry[ i ].normalChannelEntry.channel = i + 1;
    }

    /* init default scan policy */
    scanPolicy.normalScanInterval = 10000;
    scanPolicy.deterioratingScanInterval = 5000;
    scanPolicy.maxTrackFailures = 3;
    scanPolicy.BSSListSize = 4;
    scanPolicy.BSSNumberToStartDiscovery = 1;
    scanPolicy.numOfBands = 1;
    scanPolicy.bandScanPolicy[ 0 ].band = RADIO_BAND_2_4_GHZ;
    scanPolicy.bandScanPolicy[ 0 ].rxRSSIThreshold = -80;
    scanPolicy.bandScanPolicy[ 0 ].numOfChannles = 11;
    scanPolicy.bandScanPolicy[ 0 ].numOfChannlesForDiscovery = 3;
    for ( i = 0; i < 11; i++ )
    {
        scanPolicy.bandScanPolicy[ 0 ].channelList[ i ] = i + 1;
    }
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.scanType = SCAN_TYPE_NORMAL_ACTIVE;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.ETMaxNumberOfApFrames = 0;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.maxChannelDwellTime = 30000;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.minChannelDwellTime = 15000;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.bitrate = DRV_RATE_MASK_1_BARKER;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = 3;
    scanPolicy.bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.txPowerDbm = MAX_TX_POWER;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.scanType = SCAN_TYPE_NORMAL_ACTIVE;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.ETMaxNumberOfApFrames = 0;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.maxChannelDwellTime = 30000;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.minChannelDwellTime = 15000;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.bitrate = DRV_RATE_MASK_2_BARKER;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = 3;
    scanPolicy.bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.txPowerDbm = MAX_TX_POWER;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.scanType = SCAN_TYPE_NORMAL_ACTIVE;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.ETMaxNumberOfApFrames = 0;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.maxChannelDwellTime = 30000;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.minChannelDwellTime = 15000;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.bitrate = DRV_RATE_MASK_5_5_CCK;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = 3;
    scanPolicy.bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.txPowerDbm = MAX_TX_POWER;
}

void cmd_Scan_Start(ConParm_t parm[], U16 nParms)
{
    TI_StartScan( g_id_adapter, &appScanParams );
    console_printf_terminal("Application scan started.\n");
}

void cmd_Scan_Stop(ConParm_t parm[], U16 nParms)
{
    TI_StopScan( g_id_adapter );
    console_printf_terminal("Application scan stopped.\n");
}

void cmd_Scan_app_global_config(ConParm_t parm[], U16 nParms)
{
    if ( 0 == strcmp( "<empty>", (char*)parm[0].value) )
    {
        appScanParams.desiredSsid.len = 0;
        appScanParams.desiredSsid.ssidString[ 0 ] = '\0';
    }
    else
    {
    appScanParams.desiredSsid.len = strlen((char*)parm[0].value);
    memcpy( &(appScanParams.desiredSsid.ssidString), (char*)parm[0].value, appScanParams.desiredSsid.len );
    }
    appScanParams.scanType = parm[1].value;
    appScanParams.band = parm[2].value;
    appScanParams.probeReqNumber = (UINT8)parm[3].value;
    appScanParams.probeRequestRate = parm[4].value;
#ifdef TI_DBG
    appScanParams.Tid = (UINT8)parm[5].value;
    appScanParams.numOfChannels  = (UINT8)parm[6].value;
#else
    appScanParams.Tid = 0;
    appScanParams.numOfChannels = (UINT8)parm[5].value;
#endif
}

void cmd_Scan_app_channel_config(ConParm_t parm[], U16 nParms)
{
    scan_normalChannelEntry_t* pChannelEntry =
        &(appScanParams.channelEntry[ parm[0].value ].normalChannelEntry);

	if (parm[2].value < parm[3].value)
	{
		console_printf_terminal ("Max Dwell Time must be larger than or equal to Min Dwell Time...\n");
		return;
	}

    hexStr2MACAddr( (char*)parm[1].value, &(pChannelEntry->bssId) );
    pChannelEntry->maxChannelDwellTime = parm[2].value;
    pChannelEntry->minChannelDwellTime = parm[3].value;
    pChannelEntry->earlyTerminationEvent = parm[4].value;
    pChannelEntry->ETMaxNumOfAPframes = (UINT8)parm[5].value;
    pChannelEntry->txPowerDbm = (UINT8)parm[6].value;
    pChannelEntry->channel = (UINT8)parm[7].value;
}

void cmd_Scan_app_clear(ConParm_t parm[], U16 nParms)
{
    memset( &appScanParams, 0, sizeof(scan_Params_t) );
    console_printf_terminal("Application scan parameters cleared.\n");
}

void cmd_Scan_app_display(ConParm_t parm[], U16 nParms)
{
    int i;
    scan_normalChannelEntry_t* pNormalChannel;
    char bssId[18];

    bssId[17] = '\0';
    console_printf_terminal("Application Scan params:\n");
    console_printf_terminal("SSID: %s, Type: %s\n", appScanParams.desiredSsid.ssidString, scanType2Str[ appScanParams.scanType ].name);
    console_printf_terminal("Band: %s, Number of probe req:%d, probe req. rate:%s\n",
          band2Str[ appScanParams.band ].name, appScanParams.probeReqNumber, rate2StrFunc( appScanParams.probeRequestRate ) );
#ifdef TI_DBG
    console_printf_terminal("Tid :%d\n\n", appScanParams.Tid);
#else
    console_printf_terminal("\n");
#endif
    console_printf_terminal("Channel  BSS ID             Max time  Min time  ET event     ET frame num Power\n");
    console_printf_terminal("-------------------------------------------------------------------------------\n");
    for ( i = 0; i < appScanParams.numOfChannels; i++ )
    {
        pNormalChannel = &(appScanParams.channelEntry[ i ].normalChannelEntry);
        mac2HexStr( &(pNormalChannel->bssId), bssId );
        console_printf_terminal ("%2d       %s  %7d   %7d   %s%3d          %1d\n",
               pNormalChannel->channel, bssId, pNormalChannel->maxChannelDwellTime,
               pNormalChannel->minChannelDwellTime, EtEvent2StrFunc( pNormalChannel->earlyTerminationEvent ),
               pNormalChannel->ETMaxNumOfAPframes, pNormalChannel->txPowerDbm);
    }
    console_printf_terminal("\n");
}

void cmd_Scan_policy_global_config(ConParm_t parm[], U16 nParms)
{
    scanPolicy.normalScanInterval =  parm[ 0 ].value;
    scanPolicy.deterioratingScanInterval = parm[ 1 ].value;
    scanPolicy.maxTrackFailures = (UINT8)(parm[ 2 ].value);
    scanPolicy.BSSListSize = (UINT8)(parm[ 3 ].value);
    scanPolicy.BSSNumberToStartDiscovery = (UINT8)(parm[ 4 ].value);
    scanPolicy.numOfBands = (UINT8)(parm[ 5 ].value);
}

void cmd_Scan_band_global_config(ConParm_t parm[], U16 nParms)
{
    scan_bandPolicy_t* bandPolicy = &(scanPolicy.bandScanPolicy[ parm [ 0 ].value ]);

    bandPolicy->band = parm[ 1 ].value;
    bandPolicy->rxRSSIThreshold = (S8)(parm[ 2 ].value);
    bandPolicy->numOfChannlesForDiscovery = (UINT8)(parm[ 3 ].value);
    bandPolicy->numOfChannles = (UINT8)(parm[ 4 ].value);
}

void cmd_Scan_band_channel_config(ConParm_t parm[], U16 nParms)
{
    scan_bandPolicy_t* bandPolicy = &(scanPolicy.bandScanPolicy[ parm [ 0 ].value ]);

    bandPolicy->channelList[ parm[ 1 ].value ] = (UINT8)(parm[ 2 ].value);
}

void cmd_Scan_band_track_config(ConParm_t parm[], U16 nParms)
{
    scan_bandPolicy_t* bandPolicy = &(scanPolicy.bandScanPolicy[ parm [ 0 ].value ]);

	if (parm[6].value < parm[7].value)
	{
		console_printf_terminal ("Max Dwell Time must be larger than or equal to Min Dwell Time...\n");
		return;
	}

    bandPolicy->trackingMethod.scanType = parm[ 1 ].value;

    switch (bandPolicy->trackingMethod.scanType)
    {
    case SCAN_TYPE_NORMAL_ACTIVE:
    case SCAN_TYPE_NORMAL_PASSIVE:
        bandPolicy->trackingMethod.method.basicMethodParams.maxChannelDwellTime = (parm[ 6 ].value);
        bandPolicy->trackingMethod.method.basicMethodParams.minChannelDwellTime = (parm[ 7 ].value);
        bandPolicy->trackingMethod.method.basicMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->trackingMethod.method.basicMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->trackingMethod.method.basicMethodParams.probReqParams.bitrate = parm[ 9 ].value;
        bandPolicy->trackingMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = (UINT8)(parm[ 8 ].value);
        bandPolicy->trackingMethod.method.basicMethodParams.probReqParams.txPowerDbm = (UINT8)(parm[ 10 ].value);
        break;

    case SCAN_TYPE_TRIGGERED_ACTIVE:
    case SCAN_TYPE_TRIGGERED_PASSIVE:

		/* Check if valid TID */
		if (((parm[ 4 ].value) > 7) && ((parm[ 4 ].value) != 255))
		{
			console_printf_terminal ("ERROR Tid (AC) should be 0..7 or 255 instead = %d (using default = 255)\n",(parm[ 4 ].value));
			parm[ 4 ].value = 255;
		}
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.triggeringTid = (UINT8)(parm[ 4 ].value);
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.maxChannelDwellTime = (parm[ 6 ].value);
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.minChannelDwellTime = (parm[ 7 ].value);
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.bitrate = parm[ 9 ].value;
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.numOfProbeReqs = (UINT8)(parm[ 8 ].value);
        bandPolicy->trackingMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.txPowerDbm = (UINT8)(parm[ 10 ].value);
        break;

    case SCAN_TYPE_SPS:
        bandPolicy->trackingMethod.method.spsMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->trackingMethod.method.spsMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->trackingMethod.method.spsMethodParams.scanDuration = parm[ 5 ].value;
        break;

    default:
        bandPolicy->trackingMethod.scanType = SCAN_TYPE_NO_SCAN;
        break;
    }
}

void cmd_Scan_band_discover_config(ConParm_t parm[], U16 nParms)
{
    scan_bandPolicy_t* bandPolicy = &(scanPolicy.bandScanPolicy[ parm [ 0 ].value ]);

	if (parm[6].value < parm[7].value)
	{
		console_printf_terminal ("Max Dwell Time must be larger than or equal to Min Dwell Time...\n");
		return;
	}

    bandPolicy->discoveryMethod.scanType = parm[ 1 ].value;

    switch (bandPolicy->discoveryMethod.scanType)
    {
    case SCAN_TYPE_NORMAL_ACTIVE:
    case SCAN_TYPE_NORMAL_PASSIVE:
        bandPolicy->discoveryMethod.method.basicMethodParams.maxChannelDwellTime = (parm[ 6 ].value);
        bandPolicy->discoveryMethod.method.basicMethodParams.minChannelDwellTime = (parm[ 7 ].value);
        bandPolicy->discoveryMethod.method.basicMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->discoveryMethod.method.basicMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->discoveryMethod.method.basicMethodParams.probReqParams.bitrate = parm[ 9 ].value;
        bandPolicy->discoveryMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = (UINT8)(parm[ 8 ].value);
        bandPolicy->discoveryMethod.method.basicMethodParams.probReqParams.txPowerDbm = (UINT8)(parm[ 10 ].value);
        break;

    case SCAN_TYPE_TRIGGERED_ACTIVE:
    case SCAN_TYPE_TRIGGERED_PASSIVE:
		/* Check if valid TID */
		if (((parm[ 4 ].value) > 7) && ((parm[ 4 ].value) != 255))
		{
			console_printf_terminal ("ERROR Tid (AC) should be 0..7 or 255 instead = %d (using default = 255)\n",(parm[ 4 ].value));
			parm[ 4 ].value = 255;
		}
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.triggeringTid = (UINT8)(parm[ 4 ].value);
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.maxChannelDwellTime = (parm[ 6 ].value);
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.minChannelDwellTime = (parm[ 7 ].value);
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.bitrate = parm[ 9 ].value;
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.numOfProbeReqs = (UINT8)(parm[ 8 ].value);
        bandPolicy->discoveryMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.txPowerDbm = (UINT8)(parm[ 10 ].value);
        break;

    case SCAN_TYPE_SPS:
        bandPolicy->discoveryMethod.method.spsMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->discoveryMethod.method.spsMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->discoveryMethod.method.spsMethodParams.scanDuration = parm[ 5 ].value;
        break;

    default:
        bandPolicy->discoveryMethod.scanType = SCAN_TYPE_NO_SCAN;
        break;
    }
}

void cmd_Scan_band_immed_config(ConParm_t parm[], U16 nParms)
{
    scan_bandPolicy_t* bandPolicy = &(scanPolicy.bandScanPolicy[ parm [ 0 ].value ]);

	if (parm[6].value < parm[7].value)
	{
		console_printf_terminal ("Max Dwell Time must be larger than or equal to Min Dwell Time...\n");
		return;
	}

    bandPolicy->immediateScanMethod.scanType = parm[ 1 ].value;

    switch (bandPolicy->immediateScanMethod.scanType)
    {
    case SCAN_TYPE_NORMAL_ACTIVE:
    case SCAN_TYPE_NORMAL_PASSIVE:
        bandPolicy->immediateScanMethod.method.basicMethodParams.maxChannelDwellTime = (parm[ 6 ].value);
        bandPolicy->immediateScanMethod.method.basicMethodParams.minChannelDwellTime = (parm[ 7 ].value);
        bandPolicy->immediateScanMethod.method.basicMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->immediateScanMethod.method.basicMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->immediateScanMethod.method.basicMethodParams.probReqParams.bitrate = parm[ 9 ].value;
        bandPolicy->immediateScanMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = (UINT8)(parm[ 8 ].value);
        bandPolicy->immediateScanMethod.method.basicMethodParams.probReqParams.txPowerDbm = (UINT8)(parm[ 10 ].value);
        break;

    case SCAN_TYPE_TRIGGERED_ACTIVE:
    case SCAN_TYPE_TRIGGERED_PASSIVE:
		/* Check if valid TID */
		if (((parm[ 4 ].value) > 7) && ((parm[ 4 ].value) != 255))
		{
			console_printf_terminal ("ERROR Tid (AC) should be 0..7 or 255 instead = %d (using default = 255)\n",(parm[ 4 ].value));
			parm[ 4 ].value = 255;
		}
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.triggeringTid = (UINT8)(parm[ 4 ].value);
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.maxChannelDwellTime = (parm[ 6 ].value);
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.minChannelDwellTime = (parm[ 7 ].value);
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.bitrate = parm[ 9 ].value;
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.numOfProbeReqs = (UINT8)(parm[ 8 ].value);
        bandPolicy->immediateScanMethod.method.TidTriggerdMethodParams.basicMethodParams.probReqParams.txPowerDbm = (UINT8)(parm[ 10 ].value);
        break;

    case SCAN_TYPE_SPS:
        bandPolicy->immediateScanMethod.method.spsMethodParams.earlyTerminationEvent = parm[ 2 ].value;
        bandPolicy->immediateScanMethod.method.spsMethodParams.ETMaxNumberOfApFrames = (UINT8)(parm[ 3 ].value);
        bandPolicy->immediateScanMethod.method.spsMethodParams.scanDuration = parm[ 5 ].value;
        break;

    default:
        bandPolicy->immediateScanMethod.scanType = SCAN_TYPE_NO_SCAN;
        break;
    }
}

void cmd_Scan_policy_display(ConParm_t parm[], U16 nParms)
{
    int i;

    console_printf_terminal("Scan Policy:\n");
    console_printf_terminal("Normal scan interval: %d, deteriorating scan interval: %d\n",
          scanPolicy.normalScanInterval, scanPolicy.deterioratingScanInterval);
    console_printf_terminal("Max track attempt failures: %d\n", scanPolicy.maxTrackFailures);
    console_printf_terminal("BSS list size: %d, number of BSSes to start discovery: %d\n",
          scanPolicy.BSSListSize, scanPolicy.BSSNumberToStartDiscovery);
    console_printf_terminal("Number of configured bands: %d\n", scanPolicy.numOfBands);
    for ( i = 0; i < scanPolicy.numOfBands; i++ )
    {
        cmd_Scan_print_band( i );
    }
}

void cmd_Scan_print_band( int i )
{
    scan_bandPolicy_t* bandPolicy = &(scanPolicy.bandScanPolicy[ i ]);
    int j;

    console_printf_terminal("\nBand: %s\n", band2Str[ bandPolicy->band ].name);
    console_printf_terminal("RSSI Threshold: %d dBm\n", bandPolicy->rxRSSIThreshold);
    console_printf_terminal("Number of channels for each discovery interval: %d\n", bandPolicy->numOfChannlesForDiscovery);
    console_printf_terminal("\nTracking Method:\n");
    cmd_Scan_print_method( &(bandPolicy->trackingMethod) );
    console_printf_terminal("\nDiscovery Method:\n");
    cmd_Scan_print_method( &(bandPolicy->discoveryMethod) );
    console_printf_terminal("\nImmediate Scan Method:\n");
    cmd_Scan_print_method( &(bandPolicy->immediateScanMethod) );
    if ( bandPolicy->numOfChannles > 0 )
    {
        console_printf_terminal("\nChannel list: ");
        for ( j = 0; j < bandPolicy->numOfChannles; j++ )
        {
            console_printf_terminal("%3d ", bandPolicy->channelList[ j ]);
        }
        console_printf_terminal("\n");
    }
    else
        console_printf_terminal("\nNo channels defined.\n");
}

void cmd_Scan_print_method( scan_Method_t* scanMethod )
{
    console_printf_terminal("Scan type: %s\n", scanType2Str[ scanMethod->scanType ].name);
    switch (scanMethod->scanType)
    {
    case SCAN_TYPE_NORMAL_ACTIVE:
    case SCAN_TYPE_NORMAL_PASSIVE:
        cmd_Scan_print_basic_method( &(scanMethod->method.basicMethodParams) );
        break;

    case SCAN_TYPE_TRIGGERED_ACTIVE:
    case SCAN_TYPE_TRIGGERED_PASSIVE:
        cmd_Scan_print_triggered_method( &(scanMethod->method.TidTriggerdMethodParams) );
        break;

    case SCAN_TYPE_SPS:
        cmd_Scan_print_sps_method( &(scanMethod->method.spsMethodParams) );
        break;

    case SCAN_TYPE_NO_SCAN:
        break;
    }
}

void cmd_Scan_print_basic_method( scan_basicMethodParams_t* basicMethodParams )
{
    console_printf_terminal("Max channel dwell time: %d, Min channel dwell time: %d\n",
          basicMethodParams->maxChannelDwellTime, basicMethodParams->minChannelDwellTime);
    console_printf_terminal("ET condition: %s, ET number of frames: %d\n",
          EtEvent2StrFunc( basicMethodParams->earlyTerminationEvent ), basicMethodParams->ETMaxNumberOfApFrames);
    console_printf_terminal("Probe request number: %d, probe request rate: %s, TX power (Dbm/10): %d\n",
          basicMethodParams->probReqParams.numOfProbeReqs, rate2StrFunc( basicMethodParams->probReqParams.bitrate ),
          basicMethodParams->probReqParams.txPowerDbm);
}

void cmd_Scan_print_triggered_method( scan_TidTriggeredMethodParams_t* triggeredMethodParams )
{
    console_printf_terminal("Triggering Tid: %d\n", triggeredMethodParams->triggeringTid);
    cmd_Scan_print_basic_method( &(triggeredMethodParams->basicMethodParams) );
}

void cmd_Scan_print_sps_method( scan_SPSMethodParams_t* spsMethodParams )
{
    console_printf_terminal("ET condition: %s, ET number of frames: %d\n",
          EtEvent2StrFunc( spsMethodParams->earlyTerminationEvent ), spsMethodParams->ETMaxNumberOfApFrames);
    console_printf_terminal("Scan duration: %d\n", spsMethodParams->scanDuration);
}

void cmd_Scan_policy_clear(ConParm_t parm[], U16 nParms)
{
    memset( &scanPolicy, 0, sizeof(scan_Policy_t) );
    console_printf_terminal("Scan policy cleared.\n");
}

void cmd_Scan_policy_store(ConParm_t parm[], U16 nParms)
{
    TI_SetScanPolicy( g_id_adapter, (UINT8*)&scanPolicy, sizeof(scan_Policy_t) );
    console_printf_terminal("Scan policy stored.\n");
}

void cmd_Scan_get_bss_list(ConParm_t parm[], U16 nParms)
{
    bssList_t list;
    int i;

    /* get list */
    if ( (TI_RESULT_OK != TI_GetScanBssList( g_id_adapter, &list )) || (0 == list.numOfEntries) )
    {
        return;
    }

    /* console_printf_terminal list */
    console_printf_terminal("BSS List:\n");
    console_printf_terminal("%-17s  %-7s  %-6s  %-4s  %-10s\n", "BSSID", "Band", "Channel", "RSSI", "Neighbor?");
    console_printf_terminal("-----------------------------------------------------\n");
    for  ( i = 0; i < list.numOfEntries; i++ )
    {

        console_printf_terminal( "%s  %s  %-7d  %-4d  %s\n",
               print_mac_2_str(list.BSSList[ i ].BSSID.addr), band2Str[ list.BSSList[ i ].band ].name,
               list.BSSList[ i ].channel, list.BSSList[ i ].RSSI,
               (TRUE == list.BSSList[ i ].bNeighborAP ? "Yes" : "No") );
    }
}

void cmd_set_dtag_to_ac_mapping_table(ConParm_t parm[], U16 nParms)
{
    int				i;
    acTrfcType_e	dtagToAcTable[MAX_NUM_OF_802_1d_TAGS];

    for (i=0; i<MAX_NUM_OF_802_1d_TAGS; i++)
    {
        dtagToAcTable[i] = (UINT32) parm[i].value;
	}
	console_printf_terminal("Input parameters =%d, %d, %d, %d, %d, %d, %d, %d\n",
                    (UINT32) parm[0].value,
                    (UINT32) parm[1].value,
                    (UINT32) parm[2].value,
                    (UINT32) parm[3].value,
                    (UINT32) parm[4].value,
                    (UINT32) parm[5].value,
                    (UINT32) parm[6].value,
                    (UINT32) parm[7].value);

	if (TI_SetDTagToAcMappingTable(g_id_adapter, dtagToAcTable) == TI_RESULT_OK)
	{
	}
	else
	{
		console_printf_terminal ("Error: could not set tag_ToAcClsfrTable ...\n");
	}
}

void cmd_set_vad(ConParm_t parm[], U16 nParms)
{
	txDataVadTimerParams_t  pVadTimer;

	if (0 == nParms)	// GET operation
	{
		console_printf_terminal("Set VAD: \n");
		console_printf_terminal("    Parm 0 -- 1: ENABLE; 0: DISABLE\n");
		console_printf_terminal("    Parm 1 -- VAD timer duration in ms for enabling action\n");
		if (TI_GetVAD(g_id_adapter, &pVadTimer) == TI_RESULT_OK)
		{

		    console_printf_terminal("Current param values: %d, %d\n",
				 pVadTimer.vadTimerEnabled, pVadTimer.vadTimerDuration);
		}
	}
	else
	{
		pVadTimer.vadTimerEnabled  = (UINT16) parm[0].value;
		pVadTimer.vadTimerDuration = (UINT16) parm[1].value;

		if (pVadTimer.vadTimerEnabled)
		{
			console_printf_terminal("Enabling VAD timer (cycle = %d ms)\n", pVadTimer.vadTimerDuration);
		}
		else
		{
			console_printf_terminal("Disabling VAD timer\n");
		}

		if (TI_SetVAD(g_id_adapter, &pVadTimer) == TI_RESULT_OK)
		{
			console_printf_terminal("Setting VAD is done\n");
		}
	}
}


void cmd_set_qos_params(ConParm_t parm[], U16 nParms)
{
   OS_802_11_QOS_PARAMS pQosParams;

   pQosParams.acID=parm[0].value;
   pQosParams.MaxLifeTime=parm[1].value;
   pQosParams.VoiceDeliveryProtocol=0;
   pQosParams.PSDeliveryProtocol=0;

   if (nParms == 7) /* If the user has input 7 parameters, it means he gave Voice+PS delivery protocol values */
   {
      pQosParams.VoiceDeliveryProtocol=parm[5].value;
      pQosParams.PSDeliveryProtocol=parm[6].value;

      /* If this QOS config was done for a queue OTHER THAN VO, we will zero the DeliveryProtocolPsPoll parameter and notify the user */
      if ((parm[0].value != 3) && (parm[5].value != 0))
         {
            pQosParams.VoiceDeliveryProtocol = 0;
            console_printf_terminal("Since the acID is not VO, resetting VoiceDeliveryProtocol parameter to PS_NONE\n");
         }
      if ((pQosParams.VoiceDeliveryProtocol == 1) && (pQosParams.PSDeliveryProtocol == 1))
      {
            pQosParams.VoiceDeliveryProtocol = 1;
            pQosParams.PSDeliveryProtocol = 0;
            console_printf_terminal("Since the VoiceDeliveryProtocol is PS_POLL, resetting PSDeliveryProtocol to Legacy\n");
      }
   }

   if (TI_SetQosParameters(g_id_adapter, &pQosParams) == TI_RESULT_OK)
   {
      console_printf_terminal("Sent QOS params to driver...\n AC Number=%d \n MaxLifeTime=%d \n DeliveryProtocolPsPoll = %d\n PSDeliveryProtocol = %d\n",
          pQosParams.acID,
          pQosParams.MaxLifeTime,
          pQosParams.VoiceDeliveryProtocol,
          pQosParams.PSDeliveryProtocol);
   }
   else
   {
      console_printf_terminal ("Error: could not set QOS params...\n");
   }
}

void cmd_set_rxTimeOut_params(ConParm_t parm[], U16 nParms)
{
    OS_802_11_QOS_RX_TIMEOUT_PARAMS rxTimeOut;

    rxTimeOut.psPoll = parm[0].value;
    rxTimeOut.UPSD   = parm[1].value;

    if (nParms != 2)
    {
        console_printf_terminal("Please enter Rx Time Out:\n");
        console_printf_terminal("Param 0 - psPoll (0 - 65000)\n");
        console_printf_terminal("Param 1 - UPSD (1 - 65000)\n");
    }
    else
    {
       if (TI_SetQosRxTimeOut(g_id_adapter, &rxTimeOut) == TI_RESULT_OK)
       {
          console_printf_terminal("Sent QOS Rx TimeOut params to driver...\n PsPoll = %d\n UPSD = %d\n",
              rxTimeOut.psPoll,
              rxTimeOut.UPSD);
       }
       else
       {
          console_printf_terminal ("Error: could not set Rx TimeOut..\n");
       }
    }
}


void cmd_enable_rx_data_filters(ConParm_t parm[], U16 nParms)
{
    console_printf_terminal("Enabling Rx data filtering...\n");
    TI_EnableDisableRxDataFilters( g_id_adapter, TRUE );
}


void cmd_disable_rx_data_filters(ConParm_t parm[], U16 nParms)
{
    console_printf_terminal("Disabling Rx data filtering...\n");
    TI_EnableDisableRxDataFilters( g_id_adapter, FALSE );
}


void cmd_get_rx_data_filters_statistics(ConParm_t parm[], U16 nParms)
{
    TIWLAN_DATA_FILTER_STATISTICS statistics;


    console_printf_terminal("Rx data filtering statistics:\n");

    TI_GetRxDataFiltersStatistics( g_id_adapter, &statistics );

    console_printf_terminal("Unmatched packets: %u\n", statistics.UnmatchedPacketsCount);
    console_printf_terminal("Packets matching filter #1: %u\n", statistics.MatchedPacketsCount[0]);
    console_printf_terminal("Packets matching filter #2: %u\n", statistics.MatchedPacketsCount[1]);
    console_printf_terminal("Packets matching filter #3: %u\n", statistics.MatchedPacketsCount[2]);
    console_printf_terminal("Packets matching filter #4: %u\n", statistics.MatchedPacketsCount[3]);
}

void cmd_show_power_consumption_stats(ConParm_t parm[])
{
    PowerConsumptionTimeStat_t statistics;


    TI_GetPowerConsumptionStatistics( g_id_adapter, &statistics );

    console_printf_terminal("\nPower Consumption Statistics:\n");
    console_printf_terminal("-----------------------------\n");
    console_printf_terminal("activeTimeCnt_H: %u\n", statistics.activeTimeCnt_Hi);
    console_printf_terminal("activeTimeCnt_Low: %u\n", statistics.activeTimeCnt_Low);
    console_printf_terminal("elpTimeCnt_Hi: %u\n", statistics.elpTimeCnt_Hi);
    console_printf_terminal("elpTimeCnt_Low: %u\n", statistics.elpTimeCnt_Low);
    console_printf_terminal("powerDownTimeCnt_Hi: %u\n", statistics.powerDownTimeCnt_Hi);
    console_printf_terminal("powerDownTimeCnt_Low: %u\n", statistics.powerDownTimeCnt_Low);

}



static void parse_hex_string(char * pString, tiUINT8 * pBuffer, tiUINT8 * Length)
{
    char ch;
    int iter = 0;

    while ((ch = pString[iter]))
    {
        UINT8 val = ((ch >= '0' && ch <= '9') ? (ch - '0') :
                     (ch >= 'A' && ch <= 'F') ? (0xA + ch - 'A') :
                     (ch >= 'a' && ch <= 'f') ? (0xA + ch - 'a') : 0);

        /* even indexes go to the lower nibble, odd indexes push them to the */
        /* higher nibble and then go themselves to the lower nibble. */
        if (iter % 2)
            pBuffer[iter / 2] = ((pBuffer[iter / 2] << (BIT_TO_BYTE_FACTOR / 2)) | val);
        else
            pBuffer[iter / 2] = val;

        ++iter;
    }

    /* iter = 0 len = 0, iter = 1 len = 1, iter = 2 len = 1, and so on... */
    *Length = (iter + 1) / 2;
}

static void parse_binary_string(char * pString, tiUINT8 * pBuffer, tiUINT8 * Length)
{
    char ch;
    int iter = 0;

    while ((ch = pString[iter]))
    {
        UINT8 val = (ch == '1' ? 1 : 0);

        if (iter % BIT_TO_BYTE_FACTOR)
            pBuffer[iter / BIT_TO_BYTE_FACTOR] |= (val << (iter % BIT_TO_BYTE_FACTOR));
        else
            pBuffer[iter / BIT_TO_BYTE_FACTOR] = val;

        ++iter;
    }

    /* iter = 0 len = 0, iter = 1 len = 1, iter = 8 len = 1, and so on... */
    *Length = (iter + BIT_TO_BYTE_FACTOR - 1) / BIT_TO_BYTE_FACTOR;
}

void cmd_add_rx_data_filter(ConParm_t parm[], U16 nParms)
{
    tiUINT32 resultCode;
    TIWLAN_DATA_FILTER_REQUEST request;

    char * mask = (char *) parm[1].value;
    char * pattern = (char *) parm[2].value;


    request.Offset = (UINT8) parm[0].value;

    parse_binary_string(mask, request.Mask, &request.MaskLength);
    parse_hex_string(pattern, request.Pattern, &request.PatternLength);

    resultCode = TI_AddRxDataFilter(g_id_adapter, &request);


    console_printf_terminal(resultCode == RX_NO_AVAILABLE_FILTERS ? "Error: There are no available filter slots.\n" :
                            resultCode == RX_FILTER_ALREADY_EXISTS ? "Error: Filter already exists.\n" :
                            resultCode == NOK ? "Error: Could not add the filter.\n" :
                            "Filter added.\n");
}

void cmd_remove_rx_data_filter(ConParm_t parm[], U16 nParms)
{
    tiUINT32 resultCode;
    TIWLAN_DATA_FILTER_REQUEST request;

    char * mask = (char *) parm[1].value;
    char * pattern = (char *) parm[2].value;

    request.Offset = (UINT8) parm[0].value;

    parse_binary_string(mask, request.Mask, &request.MaskLength);
    parse_hex_string(pattern, request.Pattern, &request.PatternLength);

    resultCode = TI_RemoveRxDataFilter(g_id_adapter, &request);

    console_printf_terminal(resultCode == RX_FILTER_DOES_NOT_EXIST ? "Error: Filter was not found.\n" :
                            resultCode == NOK ? "Error: Could not remove the filter, removal request must be identical to add request.\n" :
                            "Filter removed.\n");
}

void cmd_MaxRxLifetime_params(ConParm_t parm[], U16 nParms)
{
    PLT_MIB_t Mib;
    UINT32 Status;
    memset(&Mib, 0, sizeof(Mib));
    Mib.aMib = PLT_MIB_dot11MaxReceiveLifetime;

	if (nParms != 1)
	{
	   console_printf_terminal("Max Rx lifetime: [0 - 4294967295(0xFFFFFFFF)]\n");

	}

	if (nParms == 0) /*Get MaxRxlifetime */
    {
        Status = TI_PLT_ReadMIB(g_id_adapter, &Mib);
        if (OK == Status)
  	       console_printf_terminal("MaxRxLifetime = %d\n", Mib.aData.MaxReceiveLifeTime);
    }
    else if (nParms == 1)
    {
        Mib.Length = sizeof(Mib.aData.MaxReceiveLifeTime);
        Mib.aData.MaxReceiveLifeTime = parm[0].value;
        if (OK != TI_PLT_WriteMIB(g_id_adapter, &Mib))
            console_printf_terminal("TI_PLT_WriteMIB failed\n");
    }
}


void cmd_add_tspec(ConParm_t parm[], U16 nParms)
{
   OS_802_11_QOS_TSPEC_PARAMS pTspecParams;
   tiUINT32 resultCode;

   pTspecParams.uUserPriority = parm[0].value;
   pTspecParams.uNominalMSDUsize = parm[1].value;
   pTspecParams.uMeanDataRate = parm[2].value;
   pTspecParams.uMinimumPHYRate = parm[3].value * 1000 * 1000;
   pTspecParams.uSurplusBandwidthAllowance = parm[4].value << 13;
   pTspecParams.uAPSDFlag = parm[5].value;

   resultCode = TI_AddTspec (g_id_adapter,&pTspecParams);

   switch (resultCode)
   {
   case OK:
      console_printf_terminal ("TSpec request sent to driver...\n uUserPriority = %d\n uNominalMSDUsize = %d\n uMeanDataRate = %d\n uMinimumPHYRate = %d\n uSurplusBandwidthAllowance = %d\n uAPSDFlag = %d\n",
               parm[0].value,
               parm[1].value,
               parm[2].value,
               parm[3].value,
               parm[4].value,
               parm[5].value);
      break;
   case TRAFIC_ADM_PENDING:
      console_printf_terminal ("Driver is still waiting for a response of previous request...\n");
      break;
   case AC_ALREADY_IN_USE:
      console_printf_terminal ("Other user priority from the same AC has already used a TSPEC...\n");
      break;
   case NOT_CONNECTED:
      console_printf_terminal ("Not connected to an AP...\n");
      break;
   case NO_QOS_AP:
      console_printf_terminal ("AP does not support QOS...\n");
      break;
   case NOK:
      console_printf_terminal ("Invalid parameter...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }

   return;
}

void cmd_get_tspec_params(ConParm_t parm[], U16 nParms)
{
   OS_802_11_QOS_TSPEC_PARAMS pTspecParams;
   tiUINT32 resultCode;

   pTspecParams.uUserPriority = parm[0].value;

   resultCode = TI_GetTspecParameters(g_id_adapter, &pTspecParams);

   switch (resultCode)
   {
   case OK:
      console_printf_terminal ("TSpec parameters retrieved:\nuUserPriority = %d\nuNominalMSDUsize = %d\nuMeanDataRate = %d\nuMinimumPHYRate = %d\nuSurplusBandwidthAllowance = %d\nuUAPSD_Flag = %d\nuMediumTime = %d\n",
               pTspecParams.uUserPriority,
               pTspecParams.uNominalMSDUsize,
               pTspecParams.uMeanDataRate,
               pTspecParams.uMinimumPHYRate,
               pTspecParams.uSurplusBandwidthAllowance,
               pTspecParams.uAPSDFlag,
               pTspecParams.uMediumTime);
      break;
   case USER_PRIORITY_NOT_ADMITTED:
      console_printf_terminal ("User Priority is not admitted...\n");
      break;
   case NOT_CONNECTED:
      console_printf_terminal ("Not connected to an AP...\n");
      break;
   case NO_QOS_AP:
      console_printf_terminal ("AP does not support QOS...\n");
      break;
   case NOK:
      console_printf_terminal ("Invalid parameter...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }

   return;
}

void cmd_delete_tspec(ConParm_t parm[], U16 nParms)
{
   OS_802_11_QOS_DELETE_TSPEC_PARAMS pDelTspecParams;
   tiUINT32 resultCode;

   pDelTspecParams.uUserPriority = parm[0].value;
   pDelTspecParams.uReasonCode = parm[1].value;

   resultCode = TI_DeleteTspec(g_id_adapter, &pDelTspecParams);

   switch (resultCode)
   {
   case OK:
      console_printf_terminal ("TSPEC Delete request sent to driver...\n uUserPriority = %d\n uReasonCode = %d\n",
               pDelTspecParams.uUserPriority,
               pDelTspecParams.uReasonCode);
      break;
   case NOT_CONNECTED:
      console_printf_terminal ("Not connected to an AP...\n");
      break;
   case NO_QOS_AP:
      console_printf_terminal ("AP does not support QOS...\n");
      break;
   case NOK:
      console_printf_terminal ("Invalid parameter...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }

}

void cmd_get_ap_qos_params(ConParm_t parm[], U16 nParms)
{
   OS_802_11_AC_QOS_PARAMS pACQosParams;
   int i = 0;
   tiUINT32 resultCode;

   pACQosParams.uAC = i;
   /* Read AC 0 parameters - just to check if connected to QOS AP etc */
   resultCode = TI_GetAPQosParameters(g_id_adapter, &pACQosParams);

   switch (resultCode)
   {
   case OK:
   console_printf_terminal ("AP QOS Parameters:\n");
         console_printf_terminal ("+----+-------------+----------+-----------+-----------+-----------+\n");
         console_printf_terminal ("| AC | AdmCtrlFlag |   AIFS   |   CwMin   |   CwMax   | TXOPLimit |\n");
         console_printf_terminal ("+----+-------------+----------+-----------+-----------+-----------+\n");


   for (i=0; i<4; i++)
      {
         pACQosParams.uAC = i;
         resultCode = TI_GetAPQosParameters(g_id_adapter, &pACQosParams);

               console_printf_terminal ("| %2d | %11d | %8d | %9d | %9d | %9d |\n",i,
               pACQosParams.uAssocAdmissionCtrlFlag,
               pACQosParams.uAIFS,
               pACQosParams.uCwMin,
               pACQosParams.uCwMax,
               pACQosParams.uTXOPLimit);
      }

         console_printf_terminal ("+----+-------------+----------+-----------+-----------+-----------+\n");
      break;
   case NOT_CONNECTED:
      console_printf_terminal ("Not connected to an AP...\n");
      break;
   case NO_QOS_AP:
      console_printf_terminal ("AP does not support QOS...\n");
      break;
   case NOK:
      console_printf_terminal ("Invalid parameter...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }

}

void cmd_get_ap_qos_capabilities(ConParm_t parm[], U16 nParms)
{
   OS_802_11_AP_QOS_CAPABILITIES_PARAMS pAPQosCapabiltiesParams;
   tiUINT32 resultCode;

   resultCode = TI_GetAPQosCapabilitesParameters(g_id_adapter, &pAPQosCapabiltiesParams);

   switch (resultCode)
   {
   case TI_RESULT_OK:
      console_printf_terminal ("AP Qos Capabilities:\n QOSFlag = %d\n APSDFlag = %d\n",pAPQosCapabiltiesParams.uQOSFlag,pAPQosCapabiltiesParams.uAPSDFlag);
      break;
   case NOT_CONNECTED:
      console_printf_terminal ("Not connected to an AP...\n");
      break;
   case NO_QOS_AP:
      console_printf_terminal ("AP does not support QOS...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }

   return;
}

void cmd_get_ac_status(ConParm_t parm[], U16 nParms)
{
   OS_802_11_AC_UPSD_STATUS_PARAMS pAcStatusParams;
   tiUINT32 resultCode;

   pAcStatusParams.uAC = parm[0].value;

   resultCode = TI_GetCurrentACStatus(g_id_adapter, &pAcStatusParams);

   switch (resultCode)
   {
   case TI_RESULT_OK:
      console_printf_terminal ("AC %d Status:\nCurrentUAPSDStatus = %d (0=PS_POLL,1=UPSD,2=PS_NONE)\nCurrentAdmissionStatus = %d(0=NOT_ADMITED,1=WAIT,2=ADMITED)\n",
               pAcStatusParams.uAC,
               pAcStatusParams.uCurrentUAPSDStatus,
               pAcStatusParams.pCurrentAdmissionStatus);
      break;
   case NOT_CONNECTED:
      console_printf_terminal ("Not connected to an AP...\n");
      break;
   case NO_QOS_AP:
      console_printf_terminal ("AP does not support QOS...\n");
      break;
   case NOK:
      console_printf_terminal ("Invalid parameters...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }

}

void cmd_get_desired_ps_mode(ConParm_t parm[], U16 nParms)
{
   OS_802_11_QOS_DESIRED_PS_MODE pDesiredPsMode;
   tiUINT32 resultCode;

   resultCode = TI_GetDesiredPsMode(g_id_adapter, &pDesiredPsMode);

   switch (resultCode)
   {
   case TI_RESULT_OK:
      console_printf_terminal ("\n\
Desired PS Mode (0=PS_POLL, 1=UPSD, 2=PS_NONE):\n\
===============================================\n\
General Desired Ps Mode  =  %d\n\
 BE_AC  Desired Ps Mode  =  %d\n\
 BK_AC  Desired Ps Mode  =  %d\n\
 VI_AC  Desired Ps Mode  =  %d\n\
 VO_AC  Desired Ps Mode  =  %d\n",
              pDesiredPsMode.uDesiredPsMode,
              pDesiredPsMode.uDesiredWmeAcPsMode[QOS_AC_BE],
              pDesiredPsMode.uDesiredWmeAcPsMode[QOS_AC_BK],
              pDesiredPsMode.uDesiredWmeAcPsMode[QOS_AC_VI],
              pDesiredPsMode.uDesiredWmeAcPsMode[QOS_AC_VO]);

      break;
   case NOK:
      console_printf_terminal ("Invalid parameters...\n");
      break;
   default:
      console_printf_terminal ("Unknown return value...\n");
      break;
   }
}



void cmd_medium_usage_threshold(ConParm_t parm[], U16 nParms)
{
   OS_802_11_THRESHOLD_CROSS_PARAMS pThresholdCrossParams;
   tiUINT32 resultCode;

   if (nParms == 3) /* If user supplied 3 parameters - this is a SET operation */
   {
       pThresholdCrossParams.uAC = parm[0].value;
       pThresholdCrossParams.uHighThreshold = parm[1].value;
       pThresholdCrossParams.uLowThreshold = parm[2].value;

      if (pThresholdCrossParams.uLowThreshold > pThresholdCrossParams.uHighThreshold)
      {
         console_printf_terminal ("Low threshold cannot be higher than the High threshold...Aborting...\n");
      }
      else
      {
           resultCode = TI_SetMediumUsageThreshold(g_id_adapter, &pThresholdCrossParams);
           if (resultCode == TI_RESULT_OK)
           {
               console_printf_terminal ("Medium usage threshold for AC %d has been set to:\n LowThreshold = %d\n HighThreshold = %d\n",
               pThresholdCrossParams.uAC,
               pThresholdCrossParams.uLowThreshold,
               pThresholdCrossParams.uHighThreshold);
           }
           else
           {
               console_printf_terminal ("Error...\n");
           }
      }

   }
   else if (nParms == 1) /* Only 1 parameter means a GET operation */
   {
      pThresholdCrossParams.uAC = parm[0].value;
      pThresholdCrossParams.uLowThreshold = 0;
      pThresholdCrossParams.uHighThreshold = 0;

      resultCode = TI_GetMediumUsageThreshold(g_id_adapter, &pThresholdCrossParams);

      console_printf_terminal ("Medium usage threshold for AC %d:\n LowThreshold = %d\n HighThreshold = %d\n",
          pThresholdCrossParams.uAC,
          pThresholdCrossParams.uLowThreshold,
          pThresholdCrossParams.uHighThreshold);
   }

}

void cmd_phy_rate_threshold(ConParm_t parm[], U16 nParms)
{
   OS_802_11_THRESHOLD_CROSS_PARAMS pThresholdCrossParams;
   tiUINT32 resultCode;

   if (nParms == 3) /* If user supplied 3 parameters - this is a SET operation */
   {
       if ((is_value_rate(parm[1].value) == FALSE) || (is_value_rate(parm[2].value) == FALSE))
       {
          console_printf_terminal ("Invalid rate - PHY rate valid values are: 1,2,5,6,9,11,12,18,24,36,48,54\n");
          return;
       }

       pThresholdCrossParams.uAC = parm[0].value;
       pThresholdCrossParams.uHighThreshold = parm[1].value;
       pThresholdCrossParams.uLowThreshold = parm[2].value;

       if (pThresholdCrossParams.uLowThreshold > pThresholdCrossParams.uHighThreshold)
       {
          console_printf_terminal ("Low threshold cannot be higher than the High threshold...Aborting...\n");
       }
       else
       {

           resultCode = TI_SetPhyRateThreshold(g_id_adapter, &pThresholdCrossParams);

           if (resultCode == TI_RESULT_OK)
           {
              console_printf_terminal ("PHY rate threshold for AC %d has been set to:\n LowThreshold = %d\n HighThreshold = %d\n",
                       pThresholdCrossParams.uAC,
                       pThresholdCrossParams.uLowThreshold,
                       pThresholdCrossParams.uHighThreshold);
           }
           else
           {
              console_printf_terminal ("Error...\n");
           }
       }
   }
   else if (nParms == 1)
   {
      pThresholdCrossParams.uAC = parm[0].value;
      pThresholdCrossParams.uLowThreshold = 0;
      pThresholdCrossParams.uHighThreshold = 0;

      resultCode = TI_GetPhyRateThreshold(g_id_adapter, &pThresholdCrossParams);

      console_printf_terminal ("PHY rate threshold for AC %d:\n",pThresholdCrossParams.uAC);
      console_printf_terminal ("LowThreshold = %s\n",print_rate((rate_e) pThresholdCrossParams.uLowThreshold));
      console_printf_terminal ("HighThreshold = %s\n",print_rate((rate_e) pThresholdCrossParams.uHighThreshold));
   }

}

void cmd_traffic_intensity_threshold(ConParm_t parm[], U16 nParms)
{
   OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS pTrafficIntensityThresholds;
   tiUINT32 resultCode;

   if (nParms == 3)
   {
      pTrafficIntensityThresholds.uHighThreshold = parm[0].value;
      pTrafficIntensityThresholds.uLowThreshold = parm[1].value;
      pTrafficIntensityThresholds.TestInterval = parm[2].value;

      if (pTrafficIntensityThresholds.uLowThreshold >= pTrafficIntensityThresholds.uHighThreshold)
      {
         console_printf_terminal ("Error: low threshold equal or greater than the high threshold...aborting...\n");
      }

      resultCode = TI_SetTrafficIntensityThresholds (g_id_adapter, &pTrafficIntensityThresholds);

      if (resultCode == TI_RESULT_OK)
      {
         console_printf_terminal ("Successfully set traffic intensity thresholds...\n");

      }
      else
      {
         console_printf_terminal ("Error: result code = %d\n",resultCode);
      }
   }
   else if (nParms == 0)
   {
      resultCode = TI_GetTrafficIntensityThresholds (g_id_adapter, &pTrafficIntensityThresholds);

      if (resultCode == TI_RESULT_OK)
      {
         console_printf_terminal ("Traffic intensity thresholds :\n HighThreshold = %d\n LowThreshold = %d\n TestInterval = %d\n",
                  pTrafficIntensityThresholds.uHighThreshold,
                  pTrafficIntensityThresholds.uLowThreshold,
                  pTrafficIntensityThresholds.TestInterval);
      }
      else
      {
         console_printf_terminal ("Error: result code = %d\n",resultCode);
      }

   }

}

void cmd_enable_traffic_events(ConParm_t parm[], U16 nParms)
{
   TI_ToggleTrafficIntensityEvents (g_id_adapter, (tiUINT32)TRUE);
   console_printf_terminal ("Traffic intensity thresholds enabled...\n");
}

void cmd_disable_traffic_events(ConParm_t parm[], U16 nParms)
{
   TI_ToggleTrafficIntensityEvents (g_id_adapter, (tiUINT32)FALSE);
   console_printf_terminal ("Traffic intensity thresholds disabled...\n");
}

void cmd_config_tx_classifier(ConParm_t parm[], U16 nParms)
{
    NWIF_CLSFR_ENTRY inParamsBuff[CLI_NUM_OF_TX_CLASFR_CON];

    UINT32 inParamsBuffLen = 0;
    UINT8 i,ic,iv=0;

    for( ic=0,iv=0; ic<CLI_NUM_OF_TX_CLASFR_CON; ic++)
    {
        inParamsBuff[ic].port = (UINT16 )parm[iv].value;
        iv++;
        inParamsBuff[ic].pri = (UINT16 )parm[iv].value;
        iv++;
        inParamsBuff[ic].ip = 0;


        inParamsBuffLen += sizeof(NWIF_CLSFR_ENTRY);

    }
    for( i=0; i<4; i++,iv++)
    {
        for(ic=0;ic<CLI_NUM_OF_TX_CLASFR_CON;ic++)
        {
            inParamsBuff[ic].ip |= parm[iv].value << i * 8;
        }
    }
}

void cmd_remove_clsfr_entry (ConParm_t parm[], U16 uParms)
{
    clsfr_tableEntry_t newUserTableEntry;
    int i;
    clsfrTypeAndSupport ClsfrType;
    tiINT32 res;

    TI_GetClsfrType(g_id_adapter, &ClsfrType );

    /* Possibly needs to be removed if we want to keep this routine working for old classifier as well */
    if (ClsfrType.oldVersionSupport == TRUE)
    {
      console_printf_terminal ("Old classifier support detected...Remove action disabled (use old classifier config)\n");
      return;
    }

    if (uParms >=2)
        newUserTableEntry.DTag = (tiUINT8) parm[1].value;

    switch(parm[0].value)
    {
        case D_TAG_CLSFR:
            console_printf_terminal("Cannot remove D_TAG classifier entry!\n");
            return;
        break;
        case DSCP_CLSFR:
            if (uParms != 3)
            {
                console_printf_terminal("DSCP_CLSFR Entry type, wrong number of parameters(too many?)\n");
                return;
            }
            newUserTableEntry.Dscp.CodePoint = (tiUINT8) parm[2].value;
            console_printf_terminal ("Removing DSCP_CLSFR classifier entry\nD-Tag = %d\nCodePoint = %d\n",newUserTableEntry.DTag,newUserTableEntry.Dscp.CodePoint);
        break;
        case PORT_CLSFR:
            if (uParms != 3)
            {
                console_printf_terminal("PORT_CLSFR Entry type, wrong number of parameters(too many?)\n");
                return;
            }
            newUserTableEntry.Dscp.DstPortNum = (tiUINT16) parm[2].value;
            console_printf_terminal ("Removing PORT_CLSFR classifier entry\nD-Tag = %d\nPort = %d\n",newUserTableEntry.DTag,newUserTableEntry.Dscp.DstPortNum);
        break;
        case IPPORT_CLSFR:
            if (uParms != 7)
            {
                console_printf_terminal("PORT_CLSFR Entry type, wrong number of parameters\n");
                return;
            }
            newUserTableEntry.Dscp.DstIPPort.DstPortNum = (tiUINT16) parm[2].value;
            newUserTableEntry.Dscp.DstIPPort.DstIPAddress = 0;
            for(i=0; i<4; i++)
                {
                    newUserTableEntry.Dscp.DstIPPort.DstIPAddress |= parm[i+3].value << i * 8;
                }
            console_printf_terminal ("Removing IPPORT_CLSFR classifier entry\nD-Tag = %d\nPort = %d\nIP = %3d.%d.%d.%d\n",
                    newUserTableEntry.DTag,
                    newUserTableEntry.Dscp.DstIPPort.DstPortNum,
                    (int)parm[3].value,(int)parm[4].value,(int)parm[5].value,(int)parm[6].value);
            break;
        default:
            console_printf_terminal("Unknown Classifier Type - Command aborted!\n");
            return;
        break;
    }

    res = TI_RemoveClassifierEntry(g_id_adapter, &newUserTableEntry);
    if (res)
    {
       console_printf_terminal ("Failed to remove classifier entry...return code = %d\n",res);
    }

}

void cmd_insert_clsfr_entry (ConParm_t parm[], U16 uParms)
{
    clsfr_tableEntry_t newUserTableEntry;
    int i;
    clsfrTypeAndSupport ClsfrType;
    tiINT32 res;

    TI_GetClsfrType(g_id_adapter, &ClsfrType );

    if (ClsfrType.oldVersionSupport == TRUE)
    {
      console_printf_terminal ("Old classifier support detected...Insert action disabled (use old classifier config)\n");
      return;
    }

    if (uParms >=2)
        newUserTableEntry.DTag = (UINT8) parm[1].value;

    switch(parm[0].value)
    {
        case D_TAG_CLSFR:
            console_printf_terminal("Cannot insert D_TAG classifier entry!\n");
            return;
        break;
        case DSCP_CLSFR:
            if (uParms != 3)
            {
                console_printf_terminal("DSCP_CLSFR Entry type, wrong number of parameters(too many?)\n");
                return;
            }
            newUserTableEntry.Dscp.CodePoint = (UINT8) parm[2].value;
            console_printf_terminal ("Inserting new DSCP_CLSFR classifier entry\nD-Tag = %d\nCodePoint = %d\n",newUserTableEntry.DTag,newUserTableEntry.Dscp.CodePoint);
        break;
        case PORT_CLSFR:
            if (uParms != 3)
            {
                console_printf_terminal("PORT_CLSFR Entry type, wrong number of parameters(too many?)\n");
                return;
            }
            newUserTableEntry.Dscp.DstPortNum = (UINT16) parm[2].value;
            console_printf_terminal ("Inserting new PORT_CLSFR classifier entry\nD-Tag = %d\nPort = %d\n",newUserTableEntry.DTag,newUserTableEntry.Dscp.DstPortNum);
        break;
        case IPPORT_CLSFR:
            if (uParms != 7)
            {
                console_printf_terminal("PORT_CLSFR Entry type, wrong number of parameters\n");
                return;
            }
            newUserTableEntry.Dscp.DstIPPort.DstPortNum = (UINT16) parm[2].value;
            newUserTableEntry.Dscp.DstIPPort.DstIPAddress = 0;
            for(i=0; i<4; i++)
                {
                    newUserTableEntry.Dscp.DstIPPort.DstIPAddress |= parm[i+3].value << i * 8;
                }
            console_printf_terminal ("Inserting new IPPORT_CLSFR classifier entry\nD-Tag = %d\nPort = %d\nIP = %3d.%d.%d.%d\n",
                    newUserTableEntry.DTag,
                    newUserTableEntry.Dscp.DstIPPort.DstPortNum,
                    (int)parm[3].value,(int)parm[4].value,(int)parm[5].value,(int)parm[6].value);
            break;
        default:
            console_printf_terminal("Unknown Classifier Type - Command aborted!\n");
            return;
        break;
    }

    res = TI_ConfigTxClassifier(g_id_adapter, sizeof(clsfr_tableEntry_t), (UINT8 *)&newUserTableEntry);
    if (res)
    {
       console_printf_terminal ("Failed to insert new classifier entry...return code = %d\n",res);
    }

}

void cmd_poll_ap_packets(ConParm_t parm[], U16 nParms)
{
   if (nParms == 0)
   {
   TI_PollApPackets (g_id_adapter);
   console_printf_terminal ("Poll AP packets cmd sent to driver...\n");
}
   else if (nParms == 1)
   {
      TI_PollApPacketsFromAC (g_id_adapter,parm[0].value);
      console_printf_terminal ("Poll AP packets (From AC %d) cmd sent to driver...\n",(int)parm[0].value);
   }

}

void cmd_modify_rate(ConParm_t parm[], U16 nParms)
{
    tiUINT32 data = 0, data1 = 0;

    if( !TI_GetDesiredRate(g_id_adapter, &data1 ) )
    {
        TI_GetCurrentRate(g_id_adapter, &data );

        console_printf_terminal("Rate: %s", print_rate(data));
        console_printf_terminal(", desired rate: %s\n", print_rate(data1));
    }
}


#if 0    /* not in use*/
void cmd_net_current_regdomain(ConParm_t parm[], U16 nParms)
{
    UNUSED(nParms);
    UNUSED(parm);

    console_printf_terminal("not implemented...\n");
}

 static named_value_t network_type_name[] = {
     { os802_11FH,                     "FH" } ,
     { os802_11DS,                     "DS" } ,
     { os802_11OFDM5,                  "OFDM5" } ,
     { os802_11OFDM24,                 "OFDM24" } ,
     { os802_11OFDM24_AND_5,           "OFDM24_AND_5" } ,
     { os802_11NetworkTypeMax,         "NetworkTypeMax" }
 };

void cmd_net_network_in_use(ConParm_t parm[], U16 nParms)
{
    OS_802_11_NETWORK_TYPE data;
    if( !nParms )
    {
        if( !TI_GetNetworkTypeInUse(g_id_adapter, &data ) )
        {
            print_available_values(network_type_name);

             console_printf_terminal("Cur.network: %d\n", data );
        }
    }
    else
        TI_SetNetworkTypeInUse(g_id_adapter, parm[0].value );
}
#endif /* if 0*/

void cmd_show_tx_power_level_table(ConParm_t parm[], U16 nParms)
{
    TIWLAN_POWER_LEVEL_TABLE powerTable;
	int i;

    if( !TI_GetTxPowerLevel(g_id_adapter, (tiCHAR*)&powerTable) )
	{
        console_printf_terminal("Power level table (Dbm/10)\n");
		for ( i = 0 ; i < TI_NUM_OF_SUB_BANDS ; i++)
		{
			console_printf_terminal("sub-band %i: %d %d %d %d\n", i,
			powerTable.uTxPower[i][0],
			powerTable.uTxPower[i][1],
			powerTable.uTxPower[i][2],
			powerTable.uTxPower[i][3]);
		}
	}
    else
	{
        console_printf_terminal("Tx Power level table ERROR !!!\n");
	}
}
void cmd_tx_power_dbm(ConParm_t parm[], U16 nParms)
{
    tiCHAR dummyData = 0;

	if (nParms == 0)
	{
		if( !TI_GetTxPowerDbm(g_id_adapter, &dummyData))
		{
			console_printf_terminal("Tx Power (Dbm/10) = %d\n", dummyData);
		}
	}
    else
    {
        if (parm[0].value > MAX_TX_POWER)
        {
            console_printf_terminal("Hey !!! You should use values between %d and %d\n", MIN_TX_POWER, MAX_TX_POWER);
            return;
        }
        /* use U8 cast to fix compile warning */
        if(! TI_SetTxPowerDbm(g_id_adapter, (U8)parm[0].value) )
        {
            console_printf_terminal("Set Tx Power in DBM/10 = %d\n", parm[0].value);
        }
    }
}


void cmd_enableDisable_802_11d(ConParm_t parm[], U16 nParms)
{
    UINT8 data = 0;
    tiINT32 result;

    result = TI_Get_802_11d(g_id_adapter, &data );
    if ( nParms == 0 )
    {
        if( result ==  TI_RESULT_OK)
        {
            console_printf_terminal("802_11d status=%d\n", data );
        }
    }
    else
    {
        result = TI_EnableDisable_802_11d(g_id_adapter, (UINT8) parm[0].value);
        if ((result != TI_RESULT_OK) && (!parm[0].value))
        {
            result = TI_Get_802_11h(g_id_adapter, &data );
            if (data)
            {
                console_printf_terminal("802_11d cannot be disabled while 802_11h is enabled!!\n" );
            }
        }
        else
        {
            console_printf_terminal("802_11d status is updated to =%d\n", parm[0].value );
        }

    }


}

void cmd_enableDisable_802_11h(ConParm_t parm[], U16 nParms)
{
    UINT8 data = 0;
    tiINT32 result;

    result = TI_Get_802_11h(g_id_adapter, &data );
    if( nParms == 0 )
    {
        if( result ==  TI_RESULT_OK)
        {
            console_printf_terminal("802_11h status=%d\n", data );
        }
    }
    else
    {
        TI_EnableDisable_802_11h(g_id_adapter, (UINT8) parm[0].value);
        if (parm[0].value)
        {
            console_printf_terminal("802_11h enables automatically 802_11d!!\n" );
        }

        console_printf_terminal("802_11h status is updated to =%d\n", parm[0].value );

    }


}

void cmd_d_Country_2_4Ie(ConParm_t parm[], U16 nParms)
{
    tiINT32 result;

    if( nParms == 0 )
    {
        UINT8   countryString[COUNTRY_STRING_LEN+1];
        result = TI_Get_countryIeFor2_4_Ghz(g_id_adapter, (UINT8**)&countryString );
        if( result ==  TI_RESULT_OK)
        {
            countryString[COUNTRY_STRING_LEN] = '\0';
            if (countryString[0] == '\0')
            {
                console_printf_terminal("802_11d Country for 2.4 GHz is not found\n");
            }
            else
            {
                console_printf_terminal("802_11d Country for 2.4 GHz is %s \n", countryString );
            }

        }
    }
    else
    {
        country_t countryWorld;

        countryWorld.elementId = COUNTRY_IE_ID;
        countryWorld.len = 6;
        memcpy( countryWorld.countryIE.CountryString,"GB ", 3);
        countryWorld.countryIE.tripletChannels[0].firstChannelNumber = 1;
        countryWorld.countryIE.tripletChannels[0].maxTxPowerLevel = 23;
        countryWorld.countryIE.tripletChannels[0].numberOfChannels = 11;
        console_printf_terminal("802_11d Start Setting GB Country for 2.4 GHz \n");

        result = TI_Set_countryIeFor2_4_Ghz(g_id_adapter, countryWorld);

        console_printf_terminal("802_11d Setting GB Country for 2.4 GHz, result=%d\n", result);

    }


}

void cmd_d_Country_5Ie(ConParm_t parm[], U16 nParms)
{
    tiINT32 result;

    if( nParms == 0 )
    {
        UINT8   countryString[COUNTRY_STRING_LEN+1];
        result = TI_Get_countryIeFor5_Ghz(g_id_adapter, (UINT8**)&countryString );
        if( result ==  TI_RESULT_OK)
        {
            countryString[COUNTRY_STRING_LEN] = '\0';
            if (countryString[0] == '\0')
            {
                console_printf_terminal("802_11d Country for 5 GHz is not found\n");
            }
            else
            {
                console_printf_terminal("802_11d Country for 5 GHz is %s\n", countryString );
            }
        }
    }
    else
    {
        country_t countryWorld;

        countryWorld.elementId = COUNTRY_IE_ID;
        countryWorld.len = 6;
        memcpy( countryWorld.countryIE.CountryString,"US ", 3);
        countryWorld.countryIE.tripletChannels[0].firstChannelNumber = 36;
        countryWorld.countryIE.tripletChannels[0].maxTxPowerLevel = 16;
        countryWorld.countryIE.tripletChannels[0].numberOfChannels = 8;
        result = TI_Set_countryIeFor5_Ghz(g_id_adapter, countryWorld);

        console_printf_terminal("802_11d Setting US Country for 5 GHz, result=%d\n", result);

    }

}

void cmd_DFS_range(ConParm_t parm[], U16 nParms)
{
    tiINT32 result;
    DFS_ChannelRange_t DFS_ChannelRange;

    if( nParms == 0 )
    {

        result = TI_Get_minMaxDfsChannels(g_id_adapter, &DFS_ChannelRange );
        if( result ==  TI_RESULT_OK)
        {
            console_printf_terminal("DFS min channel is %d, DFS max channel is %d\n",
                                    DFS_ChannelRange.minDFS_channelNum, DFS_ChannelRange.maxDFS_channelNum);
        }
    }
    else
    {
        DFS_ChannelRange.minDFS_channelNum = (UINT16) parm[0].value;
        DFS_ChannelRange.maxDFS_channelNum = (UINT16) parm[1].value;

        console_printf_terminal("Given params: min channel %d, DFS max channel %d\n",
                                parm[0].value, parm[1].value);

        result = TI_Set_minMaxDfsChannels(g_id_adapter, DFS_ChannelRange);
        if (result ==  TI_RESULT_OK)
        {
            console_printf_terminal("Setting DFS min channel %d, DFS max channel %d\n",
                                    DFS_ChannelRange.minDFS_channelNum, DFS_ChannelRange.maxDFS_channelNum);
        }
        else
        {
            console_printf_terminal("Setting DFS min channel %d, DFS max channel %d - FAILED !!\n",
                                    DFS_ChannelRange.minDFS_channelNum, DFS_ChannelRange.maxDFS_channelNum);
        }
    }

}

#if 0
void cmd_modify_tx_power_value(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 1234;
        if( !TI_GetTxPowerValue(g_id_adapter, &data) )
            console_printf_terminal(  "Tx Power val = %ld\n", data);
    }
    else
        TI_SetTxPowerValue(g_id_adapter, parm[0].value);
}
#endif

void cmd_show_regdomain_table(ConParm_t parm[], U16 nParms)
{
    UNUSED(nParms);
    UNUSED(parm);
    console_printf_terminal(  "not implemented ....\n");
}

void cmd_modify_4x_state(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiBOOL data = FALSE;
        if( !TI_Get4XState(g_id_adapter, &data ) )
            console_printf_terminal("4x state=%s\n", data ? "True" : "False" );
    }
    else    /* param <read-only!!> */
        TI_Set4XState(g_id_adapter, (BOOL) parm[0].value);
}

static named_value_t BSS_type[] =
{
    { os802_11IBSS,                  "AD-Hoc" },
    { os802_11Infrastructure,        "Infr." },
    { os802_11AutoUnknown,           "Auto" },
/*    { os802_11HighSpeedIBSS,         "HighSpeedIBSS" },*/
/*    { os802_11InfrastructureMax,     "Max" }*/
};

void cmd_modify_bss_type(ConParm_t parm[], U16 nParms)
{
    OS_802_11_NETWORK_MODE data = 0;     //TRS:MEB use correct datatype to avoid compiler warning
    if( nParms == 0 )
    {
        if( !TI_GetBSSType(g_id_adapter, &data ) )
        {
            print_available_values(BSS_type);

            console_printf_terminal("Current mode=%d\n", data );
        }
    }
    else    /* param <read-only!!> */
        TI_SetBSSType(g_id_adapter, (BOOL) parm[0].value);
}

void cmd_get_driver_state(ConParm_t parm[], U16 nParms)
{
   static char stateDesc[6][100] =
{
    "DRIVER_STATE_IDLE",
    "DRIVER_STATE_SCANNING",
    "DRIVER_STATE_SELECTING",
    "DRIVER_STATE_CONNECTING",
    "DRIVER_STATE_CONNECTED",
    "DRIVER_STATE_DISCONNECTED",
};
   driverState_e myState;

   TI_GetDriverState (g_id_adapter, &myState);
   console_printf_terminal("Driver state is %s\n", stateDesc[(UINT8)myState]);
}

void cmd_modify_ext_rates_ie(ConParm_t parm[], U16 nParms)
{
    static named_value_t ExtRatesIE[] =
    {
        { DRAFT_5_AND_EARLIER,  "5_AND_EARLIER" },
        { DRAFT_6_AND_LATER,    "6_AND_LATER" }
    };
    if( nParms == 0 )
    {
        tiUINT32 data = 1122;
        if( !TI_GetExtRatesIE(g_id_adapter, &data ) )
        {
            print_available_values(ExtRatesIE);
            console_printf_terminal("ExtRatesIE=%u\n", data  );
        }
    }
    else
        TI_SetExtRatesIE(g_id_adapter, (tiUINT32) parm[0].value);
}


/*will return RSSI*/
void cmd_get_rsii_level(ConParm_t parm[], U16 nParms)
{
   tiINT32 rssi ;
   TI_GetRSSI(g_id_adapter, &rssi);
   console_printf_terminal("\n Current RSSI : %d\n" ,rssi)  ; // TRS:WDK - add return
}


/*will return SNR ratio*/
void cmd_get_snr_ratio(ConParm_t parm[], U16 nParms)
{
   tiUINT32 snr ;
   TI_GetSNR(g_id_adapter, &snr);
   console_printf_terminal("\n Current SNR ratio : %d\n" ,snr)  ;  // TRS:WDK - add return
}

void cmd_modify_frag_threshold(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 0xfefefefe;
        if( !TI_GetFragmentThreshold(g_id_adapter, &data ) )
            console_printf_terminal("Frag. threshold=%d\n", data );
    }
    else
        TI_SetFragmentThreshold(g_id_adapter, parm[0].value);
}

void cmd_modify_short_slot(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 0xfefefefe;
        if( !TI_GetShortSlot(g_id_adapter, &data ) )
            console_printf_terminal("Short slot=%d\n", data );
    }
    else
        TI_SetShortSlot(g_id_adapter, parm[0].value);
}

void cmd_modify_rts_threshold(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 0xfefefefe;
        if( !TI_GetRTSThreshold( g_id_adapter, &data) )
            console_printf_terminal("RTSThreshold=%d\n", data );
    }
    else
        TI_SetRTSThreshold(g_id_adapter, parm[0].value);
}

void cmd_modify_preamble(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 0xfefefefe;
        if( !TI_GetShortPreamble( g_id_adapter, &data) )
            console_printf_terminal("ShortPreamble=%d\n", data );
    }
    else
        TI_SetShortPreamble(g_id_adapter, parm[0].value);
}

void cmd_modify_antenna_diversity(ConParm_t parm[], U16 nParms)
{
    TIWLAN_ANT_DIVERSITY antennaDiversityOptions;

    antennaDiversityOptions.enableRxDiversity = (UINT8)parm[0].value;
    antennaDiversityOptions.rxSelectedAntenna = (UINT8)parm[1].value;
    antennaDiversityOptions.enableTxDiversity = (UINT8)parm[2].value;
    antennaDiversityOptions.txSelectedAntenna = (UINT8)parm[3].value;
    antennaDiversityOptions.rxTxSharedAnts = (UINT8)parm[4].value;
    TI_SetAntennaDiversityParams(g_id_adapter, &antennaDiversityOptions);
    console_printf_terminal("Antenna diversity parameters sent.\n");
}

void cmd_modify_short_retry(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 0xfefefefe;
        if( !TI_GetShortRetry( g_id_adapter, &data) )
            console_printf_terminal("ShortRetry=%d\n", data );
    }
    else
        TI_SetShortRetry(g_id_adapter, parm[0].value);
}

void cmd_modify_long_retry(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        tiUINT32 data = 0xfefefefe;
        if( !TI_GetLongRetry( g_id_adapter, &data) )
            console_printf_terminal("LongRetry=%d\n", data );
    }
    else
        TI_SetLongRetry(g_id_adapter, parm[0].value);
}

void cmd_start_driver(ConParm_t parm[], U16 nParms)
{
    TI_Start( g_id_adapter );
}

void cmd_stop_driver(ConParm_t parm[], U16 nParms)
{
    UNUSED(nParms);
    UNUSED(parm);
    TI_Stop(g_id_adapter);
}
static TI_HANDLE eventRegistered[IPC_EVENT_MAX];

static named_value_t event_type[] = {
    { IPC_EVENT_ASSOCIATED,             "Associated" },
    { IPC_EVENT_DISASSOCIATED,          "Disassociated"  },
    { IPC_EVENT_LINK_SPEED,             "LinkSpeed" },
    { IPC_EVENT_AUTH_SUCC,              "Authentication Success" },
    { IPC_EVENT_SCAN_COMPLETE,          "ScanComplete" },
    { IPC_EVENT_TIMEOUT,                "Timeout" },
    { IPC_EVENT_CCKM_START,             "CCKM_Start" },
    { IPC_EVENT_MEDIA_SPECIFIC,         "Media_Specific" },
    { IPC_EVENT_EAPOL,                  "EAPOL" },
    { IPC_EVENT_BOUND,                  "Bound" },
    { IPC_EVENT_UNBOUND,                "Unbound" },
    { IPC_EVENT_PREAUTH_EAPOL,          "PreAuth EAPOL"},
/*  { IPC_EVENT_PER,                    "PER" },*/
    { IPC_EVENT_LOW_SNR,               "Low SNR" },
    { IPC_EVENT_LOW_RSSI,               "Low RSSI" },
    { IPC_EVENT_TSPEC_STATUS,           "IPC_EVENT_TSPEC_STATUS" },
    { IPC_EVENT_TSPEC_RATE_STATUS,      "IPC_EVENT_TSPEC_RATE_STATUS" },
    { IPC_EVENT_MEDIUM_TIME_CROSS,      "IPC_EVENT_MEDIUM_TIME_CROSS" },
    { IPC_EVENT_ROAMING_COMPLETE,       "ROAMING_COMPLETE"},
    { IPC_EVENT_EAP_AUTH_FAILURE,       "EAP-FAST/LEAP Auth Failed"},
    { IPC_EVENT_WPA2_PREAUTHENTICATION, "IPC_EVENT_WPA2_PREAUTHENTICATION" },
    { IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED, "IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED" },
    { IPC_EVENT_BT_COEX_MODE, 	"IPC_EVENT_BT_COEX_MODE" },

};


void cmd_events_register(ConParm_t parm[], U16 nParms)
{
    tiUINT32 event;
    IPC_EVENT_PARAMS pEvent;

   if( nParms )
   {
        event = (tiUINT32)parm[0].value;

        if (events_mask & (1 << event))
        {
#ifndef _WINDOWS
            console_printf_terminal("cmd_events_register, event is already enabled! (%d)\n", events_mask);
#endif
        }
        else
   {
        pEvent.uEventType       = (tiUINT32)parm[0].value;
        pEvent.uDeliveryType      = DELIVERY_PUSH;
        pEvent.pfEventCallback = cli_receive_ev;
#ifdef _WINDOWS
#endif
        if( !TI_RegisterEvent(g_id_adapter, &pEvent) )
            {
            eventRegistered[pEvent.uEventType] = pEvent.uEventID;

                events_mask |= (1 << event);
#ifdef _WINDOWS
#endif
            }
        }
        }
    else
        {
#ifdef _WINDOWS
#else
		print_available_values(event_type);
#endif
    }

        }
void cmd_events_unregister(ConParm_t parm[], U16 nParms)
{
    tiUINT32 event;
    IPC_EVENT_PARAMS pEvent;

   if( nParms )
   {
        event = (tiUINT32)parm[0].value;
#ifdef _WINDOWS
#else
        pEvent.uEventType = event;
        pEvent.uEventID = eventRegistered[pEvent.uEventType];
        TI_UnRegisterEvent(g_id_adapter, &pEvent);

        events_mask &= ~(1 << event);
#endif
    }
    else
    {
#ifdef _WINDOWS
#else
        print_available_values(event_type);
#endif
    }
}

void cmd_get_selected_bssid_info(ConParm_t parm[], U16 nParms)
{
   OS_802_11_BSSID_EX myInfo;
   TI_GetSelectedBSSIDInfo(g_id_adapter, &myInfo);

   console_printf_terminal("Selected BSSID Info:\n");
   console_printf_terminal("--------------------\n");
   console_printf_terminal("SSID: %s\n", get_ssid_string(&myInfo.Ssid));
   console_printf_terminal("BSSID: %02x.%02x.%02x.%02x.%02x.%02x\n",
          myInfo.MacAddress[0], myInfo.MacAddress[1], myInfo.MacAddress[2], myInfo.MacAddress[3], myInfo.MacAddress[4], myInfo.MacAddress[5] );
}

int parseBssidIe(OS_802_11_BSSID_EX * bssid)
{
    OS_802_11_VARIABLE_IEs *pData;
    dot11_WME_PARAM_t *  qosParams;

    int length;
    int retval = 0;
    /* console_printf_terminal("parseBssidIe,IElength=%d \n",bssid->IELength);*/
    for (
            length =sizeof(OS_802_11_FIXED_IEs) , pData = (OS_802_11_VARIABLE_IEs*) ((char*)bssid->IEs + sizeof(OS_802_11_FIXED_IEs));
            length < (int)(bssid->IELength-3);
            length += (pData->Length +2),pData =  (OS_802_11_VARIABLE_IEs*)((char*)bssid->IEs  + length)
        )
    {
        /* console_printf_terminal("ElementID=%d pData=%x length=%d length1=%d\n",pData->ElementID,pData,pData->Length,length);*/
        if (pData->ElementID == DOT11_WME_ELE_ID)
        {
            qosParams = (dot11_WME_PARAM_t *)pData;
            /* console_printf_terminal("OUIType=%x OUI =%x %x %x \n",qosParams->OUIType,qosParams->OUI[0],qosParams->OUI[1],qosParams->OUI[2]);*/
            if (qosParams->OUIType == dot11_WME_OUI_TYPE)
            {
                retval |= dot11_WME_OUI_TYPE;
            }


        }
    }


    return retval;
}

void cmd_bssid_list(ConParm_t parm[], U16 nParms)
{

    OS_802_11_BSSID_EX BssIdInfo;
    TI_GetSelectedBSSIDInfo(g_id_adapter, &BssIdInfo);
    get_bssid_list(parm, nParms, FALSE , &BssIdInfo );
}

void cmd_Full_bssid_list(ConParm_t parm[], U16 nParms)
{
    OS_802_11_BSSID_EX BssIdInfo;
    TI_GetSelectedBSSIDInfo(g_id_adapter, &BssIdInfo);
    get_bssid_list(parm, nParms, TRUE , &BssIdInfo);
}

/*When beacon filter is activated, the current RSSI of the connection with the AP will be displayed despite no beacons are
 *passed up to the driver*/
static void get_bssid_list(ConParm_t parm[], U16 nParms, BOOL fullBssidList , OS_802_11_BSSID_EX *pBssid)
{
    OS_802_11_BSSID_LIST_EX *list;/* = (OS_802_11_BSSID_LIST_EX *) data; */
    OS_802_11_BSSID_EX *bssid;
    tiUINT32 number_items, index;
    char buffer[8] ;
    int Qos = 0;
	BOOL isConnectedAp = FALSE ;   //TRS:MEB move this line earlier to avoid compile error
    buffer[0] ='\0';

    UNUSED(nParms);
    UNUSED(parm);

    if (fullBssidList)
    {
        if( TI_GetFullBSSIDList(g_id_adapter, &list) || !list )
            return ;
    }
    else
    {
    if( TI_GetBSSIDList(g_id_adapter, &list) || !list )
        return ;
    }


    bssid = &list->Bssid[0];
    number_items = list->NumberOfItems;

    console_printf_terminal("BssId List: Num=%u\n", number_items );

    if( number_items )
    {
        console_printf_terminal("%17s %7s %4s %5s %7s  %10s %s\n", "MAC", "Privacy", "Rssi", "Infra", "Channel","Qos   ", "SSID");
        while (number_items)
        {
            Qos = parseBssidIe(bssid);

/*          console_printf_terminal("Qos=%d\n",Qos);*/
            if (Qos & dot11_WME_OUI_TYPE)
            {
                strcpy(buffer, "WME ");
            }

            if (Qos == 0)
            {
                strcpy(buffer, "NONE ");
            }

            if (( 0 == memcmp(pBssid->MacAddress ,bssid->MacAddress,sizeof(OS_802_11_MAC_ADDRESS)) ) &&
				(pBssid->NetworkTypeInUse == bssid->NetworkTypeInUse))
            {
                /*bssid->Rssi = staRssi ;*/
                isConnectedAp = TRUE ;
            }
            else
            {
                isConnectedAp = FALSE ;
            }

            console_printf_terminal("%s %s %7u %4d %5d %7d %10s %s\n",
                    ( TRUE == isConnectedAp)?"*":" " ,
                    print_mac_2_str(bssid->MacAddress),
                    bssid->Privacy, bssid->Rssi,
                    bssid->InfrastructureMode, Freq2Chan(bssid->Configuration.Union.channel),
                    buffer,
                    get_ssid_string(&bssid->Ssid) );

            if (fullBssidList)
            {
                console_printf_terminal("   TSF 0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x\n",
                      bssid->IEs[0], bssid->IEs[1], bssid->IEs[2], bssid->IEs[3],
                      bssid->IEs[4], bssid->IEs[5], bssid->IEs[6], bssid->IEs[7]);
                console_printf_terminal("   BeaconInterval 0x%02x%02x\n",  bssid->IEs[9], bssid->IEs[8]);
                console_printf_terminal("   Capabilities   0x%02x%02x\n",  bssid->IEs[10], bssid->IEs[11]);
                console_printf_terminal("   Variable IEs:\n");
                for (index=12; index<bssid->IELength; index++)
                {
                    if ((index-12)%8 == 0)
                    {
                        console_printf_terminal("\n   ");
                    }
                    console_printf_terminal("0x%02x ",bssid->IEs[index]);

                }
                console_printf_terminal("\n");
            }
            bssid = (OS_802_11_BSSID_EX *) (((char *) bssid) + bssid->Length);
            number_items--;
        }
        console_printf_terminal("Infra.mode:");
        print_available_values(BSS_type);
    }

    free(list);
}

#ifdef _WINDOWS
#else
PACKED_STRUCT( OS_802_11_BSSID_EX_TEMP,
	tiUINT32                  Length;
	OS_802_11_MAC_ADDRESS     MacAddress;
    PACKED_UNION(Union,
        tiUINT8  Reserved[2];
        tiUINT16 Capabilities;
    );
	OS_802_11_SSID            Ssid;
	tiUINT32                  Privacy;
	OS_802_11_RSSI            Rssi;
	OS_802_11_NETWORK_TYPE    NetworkTypeInUse;
	OS_802_11_CONFIGURATION   Configuration;
	OS_802_11_NETWORK_MODE    InfrastructureMode;
	OS_802_11_RATES_EX        SupportedRates;
	tiUINT32                  IELength;
	tiUINT8                   IEs[MAX_BEACON_BODY_LENGTH+sizeof(OS_802_11_FIXED_IEs)];
);
#endif


void cmd_FullPrimaryBbssid(ConParm_t parm[], U16 nParms)
{
    OS_802_11_BSSID_EX_TEMP bssid;
    OS_802_11_BSSID_EX_TEMP *pBssid = &bssid;
    UINT32  index;
    char buffer[8] ;
    int Qos = 0;
    buffer[0] ='\0';

    UNUSED(nParms);
    UNUSED(parm);

    memset(pBssid, 0, sizeof(OS_802_11_BSSID_EX));
    pBssid->Length = sizeof(OS_802_11_BSSID_EX) + sizeof(OS_802_11_FIXED_IEs) + MAX_BEACON_BODY_LENGTH;
    if( TI_GetPrimaryBSSIDInfo(g_id_adapter, (OS_802_11_BSSID_EX*)pBssid))
            return ;

    console_printf_terminal("Primary BssId: Length = %d, IELength=%d \n" , pBssid->Length, pBssid->IELength);

    if (pBssid->Length > sizeof(OS_802_11_BSSID_EX) + sizeof(OS_802_11_FIXED_IEs) + MAX_BEACON_BODY_LENGTH)
    {
        console_printf_terminal("Error - Length = %d is too long!!\n", pBssid->Length);
        return;
    }
    if (pBssid->Length < pBssid->IELength)
    {
        console_printf_terminal("Error - IELength = %d is too long!!\n", pBssid->IELength);
        pBssid->IELength = 50;
        print_mac_2_str(pBssid->MacAddress),

        console_printf_terminal("SSID len=%d\n", pBssid->Ssid.SsidLength);

        return;
    }
    console_printf_terminal("%17s %7s %4s %5s %7s  %10s %s\n", "MAC", "Privacy", "Rssi", "Infra", "Channel","Qos   ", "SSID");
    Qos = parseBssidIe((OS_802_11_BSSID_EX*)pBssid);

    if (Qos & dot11_WME_OUI_TYPE)
    {
        strcpy(buffer, "WME ");
    }

    if (Qos == 0)
    {
        strcpy(buffer, "NONE ");
    }

    console_printf_terminal("%s %7u %4d %5d %7d %10s %s\n",
            print_mac_2_str(pBssid->MacAddress),
            pBssid->Privacy, pBssid->Rssi,
            pBssid->InfrastructureMode, Freq2Chan(pBssid->Configuration.Union.channel),
            buffer,
            get_ssid_string(&pBssid->Ssid) );

    {
        console_printf_terminal("   TSF 0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x\n",
              pBssid->IEs[0], pBssid->IEs[1], pBssid->IEs[2], pBssid->IEs[3],
              pBssid->IEs[4], pBssid->IEs[5], pBssid->IEs[6], pBssid->IEs[7]);
        console_printf_terminal("   BeaconInterval 0x%02x%02x\n",  pBssid->IEs[9], pBssid->IEs[8]);
        console_printf_terminal("   Capabilities   0x%02x%02x\n",  pBssid->IEs[10], pBssid->IEs[11]);
        console_printf_terminal("   Variable IEs:\n");
        for (index=12; index<pBssid->IELength; index++)
        {
            if ((index-12)%8 == 0)
            {
                console_printf_terminal("\n   ");
            }
            console_printf_terminal("0x%02x ",pBssid->IEs[index]);

        }
        console_printf_terminal("\n");
    }
    console_printf_terminal("\nInfra.mode:");
    print_available_values(BSS_type);
}

void cmd_file_load(ConParm_t parm[], U16 nParms)
{
    consoleRunScript((char *) parm[0].value);
}

#ifdef TI_DBG

struct report_bit_desc_t
{
    int index;
    char *desc;
};

struct report_bit_desc_t report[] =
{

    /* Set 0 */
    { CONFIG_MGR_MODULE_LOG,        "CONFIG_MGR" },
    { SME_SM_MODULE_LOG,            "SME_SM" },
    { SITE_MGR_MODULE_LOG,          "SITE_MGR" },
    { CONN_MODULE_LOG,              "CONN" },
    { MLME_SM_MODULE_LOG,           "MLMEE" },
    { AUTH_MODULE_LOG,              "AUTH" },
    { ASSOC_MODULE_LOG,             "ASSOC" },
    { RX_DATA_MODULE_LOG,           "RX_DATA" },
    { TX_DATA_MODULE_LOG,           "TX_DATA" },
    { CTRL_DATA_MODULE_LOG,         "CTRL_DATA" },
    { RSN_MODULE_LOG,               "RSN" },
    { HAL_RX_MODULE_LOG,            "HAL_RX" },
    { HAL_TX_MODULE_LOG,            "HAL_TX" },
    { HAL_CTRL_MODULE_LOG,          "HAL_CTRL" },
    { HAL_SECURITY_MODULE_LOG,      "HAL_SECUR" },
    { MEM_MGR_MODULE_LOG,           "MEM_MGR" },
    { REPORT_MODULE_LOG,            "REPORT" },
    { SITE_UPDATE_MODULE_LOG,       "SITE_UPDATE" },
    { REGULATORY_DOMAIN_MODULE_LOG, "REG_DOMAIN" },
    { MEASUREMENT_MNGR_MODULE_LOG,  "MEASUREMENT_MNGR" },
    { MEASUREMENT_SRV_MODULE_LOG,   "MEASUREMENT_SRV" },
    { SOFT_GEMINI_MODULE_LOG,       "SOFT_GEMINI" },
    { SC_MODULE_LOG,                "SC (Switch Channel)" },
    { EXC_MANAGER_MODULE_LOG,       "EXC_MANAGER" },
    { ROAMING_MANAGER_MODULE_LOG,   "ROAMING_MANAGER" },
    { QOS_MANAGER_MODULE_LOG,       "QOS_MANAGER" },
    { TRAFFIC_ADM_CTRL_MODULE_LOG,  "TRAFFIC_ADM_CTRL" },
    { POWER_MANAGER_MODULE_LOG,     "POWER_MANAGER" },
    { POWER_CONTROL_MODULE_LOG,     "POWER_CONTROL" },
    { POWER_SERVER_MODULE_LOG,      "POWER_SERVER" },
    { ELP_MODULE_LOG,               "ELP" },
    { SCR_MODULE_LOG,               "SCR" },
    { SCAN_SRV_MODULE_LOG,          "SCAN_SRV" },
    { SCAN_CNCN_MODULE_LOG,         "SCAN_CNCN" },
    { SCAN_MNGR_MODULE_LOG,         "SCAN_MNGR" },
    { GWSI_ADAPT_MODULE_LOG,        "GWSI_ADAPT" },
    { GWSI_ADAPT_CB_MODULE_LOG,     "GWSI_ADAPT_CB" },
    { CORE_ADAPT_MODULE_LOG,        "CORE_ADAPT" },
    { TX_HW_QUEUE_MODULE_LOG,       "TX HW QUEUE" },
    { TX_CTRL_BLK_MODULE_LOG,       "TX CTRL BLK" },
    { TX_RESULT_MODULE_LOG,         "TX RESULT" },
    { TNETW_IF_MODULE_LOG,          "TNETW IF" },
    { TNETW_ARBITER_MODULE_LOG,     "TNETW ARBITER" },
    { CURR_BSS_MODULE_LOG,          "CURR_BSS" },
    { FW_EVENT_MODULE_LOG,          "FW_EVENT" },
    { CMD_MBOX_MODULE_LOG,          "CMD_MBOX" },
	{ CMDQUEUE_MODULE_LOG,          "CMD_QUEUE" },
    { EVENT_MBOX_MODULE_LOG,        "EVENT_MBOX"},
    { TNETW_DRV_MODULE_LOG,         "TNETW DRV" },
	{ TNETW_XFER_MODULE_LOG,        "TX XFER" },
    { RECOVERY_MGR_MODULE_LOG,      "RECOVERY MGR" },
    { RECOVERY_CTRL_MODULE_LOG,     "RECOVERY CTRL" },
    { HW_INIT_MODULE_LOG,           "HW INIT" }

};


struct rep_severity_level_t {
    U8 level;
    char *desc;
};

static struct rep_severity_level_t report_severity_level[] = {
    { 0,                          "----"           },
    { WLAN_SEVERITY_INIT,         "INIT",          },
    { WLAN_SEVERITY_INFORMATION,  "INFORMATION",   },
    { WLAN_SEVERITY_WARNING,      "WARNING",       },
    { WLAN_SEVERITY_ERROR,        "ERROR",         },
    { WLAN_SEVERITY_FATAL_ERROR,  "FATAL_ERROR",   },
    { WLAN_SEVERITY_SM,           "SM",            },
    { WLAN_SEVERITY_CONSOLE,      "CONSOLE",       },
    { WLAN_SEVERITY_DEBUG_RX,     "DEBUG RX",      },
    { WLAN_SEVERITY_DEBUG_TX,     "DEBUG TX",      },
    { WLAN_SEVERITY_DEBUG_CONTROL,"DEBUG CONTROL", },
    { WLAN_SEVERITY_GWSI_RECORDING,"GWSI RECORDING"}
};

static void print_report_module_desc(void)
{
    int i;
    tiUINT8 module_table[WLAN_MAX_LOG_MODULES];

    if (!TI_GetReportModule(g_id_adapter, module_table))
    {
        console_printf_terminal("-------------------------------\n");
        console_printf_terminal("%.5s\tState\t %s\n", "Index", "Desc");

        for( i = 0; i < SIZE_ARR(report); i++)
        {
            /* Check if there is string content (the first character is not ZERO) */
            if( report[i].desc[0] )
            {
                console_printf_terminal("%3d\t%c\t%s\n", report[i].index, (module_table[i] == '1') ? '+' : ' ', report[i].desc );
            }
        }
    }
    else
    {
        console_printf_terminal("Error reading the report table form the driver\n");
    }
}

void cmd_report_add(ConParm_t parm[], U16 nParms)
{
    tiUINT8 module_table[WLAN_MAX_LOG_MODULES];

    if( nParms != 1)
    {
        print_report_module_desc();
        console_printf_terminal( "* Use '%d' (max index) to set all table.\n", WLAN_MAX_LOG_MODULES);
    }
    else if(!TI_GetReportModule(g_id_adapter, module_table))
    {
        if (parm[0].value == WLAN_MAX_LOG_MODULES)
        {
            memset(module_table, '1', sizeof(module_table));
        }
        else if(parm[0].value < WLAN_MAX_LOG_MODULES)
        {
            module_table[parm[0].value] = '1';
        }
        TI_SetReportModule(g_id_adapter, module_table);
    }
}

void cmd_report_clear(ConParm_t parm[], U16 nParms)
{
    tiUINT8 module_table[WLAN_MAX_LOG_MODULES + 1];

    if( nParms != 1)
    {
        print_report_module_desc();
        console_printf_terminal( "* Use '%d' (max index) to clear all table.\n", WLAN_MAX_LOG_MODULES);
    }
    else if(!TI_GetReportModule(g_id_adapter, module_table))
    {
        if (parm[0].value == WLAN_MAX_LOG_MODULES)
        {
            memset(module_table, '0', WLAN_MAX_LOG_MODULES);
        }
        else if (parm[0].value < WLAN_MAX_LOG_MODULES)
        {
            module_table[parm[0].value] = '0';
        }
        TI_SetReportModule(g_id_adapter, module_table);
    }
}

void cmd_report_set(ConParm_t parm[], U16 nParms)
{
    U8 *ModuleTable = (U8 *)parm[0].value;

    if( nParms != 1)
    {
        print_report_module_desc();
    }
    else
    {
#ifdef _WINDOWS
#endif /* _WINDOWS */

        TI_SetReportModule(g_id_adapter, ModuleTable);
    }
}

void cmd_hw_register(ConParm_t parm[], U16 nParms)
{
    tiUINT32 data;

#ifndef _WINDOWS
        usleep(10 * 1000);
#elif defined (_WINDOWS)
#endif

    if( nParms == 1 )
    {
        if( !TI_hwReadRegister(g_id_adapter, parm[0].value, &data ) )
        {
#ifdef _WINDOWS
#endif
        }
    }
    else
    {
        TI_hwWriteRegister(g_id_adapter, parm[0].value, parm[1].value );
    }
}

void cmd_debug_driver_print(ConParm_t parm[], U16 nParms)
    {
        tiUINT32 func_id = ( nParms > 0 ) ? parm[0].value : 0;
        tiUINT32 opt_param = ( nParms > 1 ) ? parm[1].value : 0;
        tiUINT32 buf[2] = { func_id, opt_param };

        console_printf_terminal("DRV_PRINT: FUNC:%u, PARAM:%u\n", func_id, opt_param);
        TI_DisplayStats(g_id_adapter, (tiUINT8 *) buf, sizeof(buf) );
        /* tiwlan_driver_debug_print( g_drv_name, func_id, opt_param ); */
}

void cmd_debug_buffer_put(ConParm_t parm[], U16 nParms)
    {
        tiUINT32 func_id = ( nParms > 0 ) ? parm[0].value : 0;
        tiUINT32 opt_param = ( nParms > 1 ) ? parm[1].value : 0;
        tiUINT8 buf[260];  /* no more then 256 + func id */

        if (opt_param == 0)
            return;
        *(tiUINT32*) buf = func_id;
        memcpy (buf + sizeof(func_id),(char *)opt_param,strlen((char *)opt_param));

        console_printf_terminal("cmd_debug_buffer_put: FUNC:%u, PARAM:%u\n", func_id, opt_param);
        TI_DisplayStats(g_id_adapter, (tiUINT8 *) buf, strlen((char *)opt_param) +  sizeof(func_id));
        /* tiwlan_driver_debug_print( g_drv_name, func_id, opt_param ); */
}

static void print_severity_table(tiUINT8 *pTable)
{
    int i;

    console_printf_terminal("Severity:\n");
    console_printf_terminal("-------------------------------\n");
    console_printf_terminal("%14s\tState\t%s\n", "Severity level", "Desc");

    for( i=1; i<SIZE_ARR(report_severity_level); i++ )
    {
        console_printf_terminal("%d\t%c\t%s\n", report_severity_level[i].level, (pTable[i] == '1') ? '+' : ' ',report_severity_level[i].desc );
    }

    console_printf_terminal( "* Use '0' to clear all table.\n");
    console_printf_terminal( "* Use '%d' (max index) to set all table.\n", SIZE_ARR(report_severity_level));
}

void cmd_report_severity_table(ConParm_t parm[], U16 nParms)
{
    U8 *pSeverityTable = (U8 *)parm[0].value;
    tiUINT8 SeverityTable[WLAN_MAX_SEVERITIES];
#ifdef _WINDOWS
#endif /* _WINDOWS */
    if( nParms != 1)
    {
        if (!TI_GetReportSeverity( g_id_adapter, &SeverityTable[0]))
        {
            print_severity_table(SeverityTable);
        }
    }
    else
    {
        TI_SetReportSeverity(g_id_adapter, pSeverityTable);
    }
}

void cmd_report_severity_level(ConParm_t parm[], U16 nParms)
{
    tiUINT8 SeverityTable[WLAN_MAX_SEVERITIES];

    /* Get the current report severity */
    if (!TI_GetReportSeverity( g_id_adapter, &SeverityTable[0]))
    {
        if(nParms == 0)
        {
            /* Parameters error - print the current table values */
            print_severity_table(SeverityTable);
        }
        else
        {
            if (parm[0].value == 0)
            {
                /* Disable all severity levels */

                memset(SeverityTable, (int)('0'), sizeof(SeverityTable));

                TI_SetReportSeverity(g_id_adapter, SeverityTable);
            }
            else if (parm[0].value == SIZE_ARR(report_severity_level))
            {
                /* Enable all severity levels */

                memset(SeverityTable, (int)('1'), sizeof(SeverityTable));

                TI_SetReportSeverity(g_id_adapter, SeverityTable);
            }
            else if (parm[0].value < SIZE_ARR(report_severity_level))
            {
                console_printf_terminal("Toggle severity level %#lx\n", parm[0].value);

                if (SeverityTable[parm[0].value] == '1')
                {
                    /* The level is enabled - Disable it */
                    SeverityTable[parm[0].value] = '0';
                }
                else
                {
                    /* The bit is disabled - Enable it */
                    SeverityTable[parm[0].value] = '1';
                }

                TI_SetReportSeverity(g_id_adapter, SeverityTable);
            }
            else
            {
                console_printf_terminal("invalid level value: %#lx\n", parm[0].value );
            }
        }
    }
    else
    {
        console_printf_terminal("Error retriving the severity table from the driver\n");
    }
}


#ifdef DRIVER_PROFILING

void cmd_profile_report(ConParm_t parm[], U16 nParms)
{
    TI_ProfileReport( g_id_adapter );
}


void cmd_profile_cpu_estimator_command(ConParm_t parm[], U16 nParms)
{
    /* reset or stop command */
    if (nParms == 1) {
        //printf("cpu_profile_cpu_estimator: param[0] = %d\n", (tiUINT8)parm[0].value);
        TI_CpuEstimatorCommand(g_id_adapter, (tiUINT8)parm[0].value, 0);
    }
    else /* start command */
    {
        //printf("cpu_profile_cpu_estimator: param[0] = %d, param[1] = %d\n",(tiUINT8)parm[0].value,(tiUINT32)parm[1].value);
        TI_CpuEstimatorCommand(g_id_adapter, (tiUINT8)parm[0].value,(tiUINT32)parm[1].value);
    }
}

#endif

void cmd_report_os_dbg_state(ConParm_t parm[], U16 nParms)
{
    UINT32 dwOsDbgState;

    if(nParms == 0) {
        if (TI_GetOsDbgState( g_id_adapter, &dwOsDbgState) == TI_RESULT_OK) {
            console_printf_terminal("OsDbgState %d (0x%08X)\n", dwOsDbgState, dwOsDbgState);
        }
        else {
            console_printf_terminal("Error retriving the OsDbgState from the driver\n");
        }
    }
    else {
        TI_SetOsDbgState(g_id_adapter, parm[0].value);
    }
}

#endif /* define TI_DBG */

void cmd_privacy_auth(ConParm_t parm[], U16 nParms)
{
    if( nParms )
    {
        TI_SetAuthenticationMode( g_id_adapter, (tiUINT32)parm[0].value );
        /*console_printf_terminal("CLI-AuthenticationMode: - %x",(tiUINT32)parm[0].value);*/
    }
    else
    {
        static named_value_t auth_mode_type[] = {
            { os802_11AuthModeOpen,             "Open"      },
            { os802_11AuthModeShared,           "Shared"    },
            { os802_11AuthModeAutoSwitch,       "AutoSwitch"},
            { os802_11AuthModeWPA,              "WPA"       },
            { os802_11AuthModeWPAPSK,           "WPAPSK"    },
            { os802_11AuthModeWPANone,          "WPANone"   },
            { os802_11AuthModeWPA2,             "WPA2"      },
            { os802_11AuthModeWPA2PSK,          "WPA2PSK"   },

            /*{ os802_11AuthModeMax,              "Max"       }*/
        };
        OS_802_11_AUTHENTICATION_MODE data;

        if( !TI_GetAuthenticationMode( g_id_adapter, &data ) )
        {
            print_available_values(auth_mode_type);
            console_printf_terminal("AuthenticationMode=%d\n", data );
        }
    }
}

void cmd_privacy_eap(ConParm_t parm[], U16 nParms)
{
    if( nParms )
    {
        TI_SetEAPType( g_id_adapter, (OS_802_11_EAP_TYPES) parm[0].value );
        TI_SetEAPTypeDriver( g_id_adapter, (OS_802_11_EAP_TYPES) parm[0].value );
    }
    else
    {
        static named_value_t eap_type[] = {
            { OS_EAP_TYPE_GENERIC_TOKEN_CARD,   "TOKEN" },
            { OS_EAP_TYPE_TLS,                  "TLS"   },
            INCLUDE_EXC_TYPE_NAMES
            { OS_EAP_TYPE_TTLS,                 "TTLS"  },
            { OS_EAP_TYPE_PEAP,                 "PEAP"  },
            {OS_EAP_TYPE_MS_CHAP_V2,            "CHAP"  }
        };
/*temp_closed
        OS_802_11_EAP_TYPES data;

        if( !TI_GetEAPType( g_id_adapter, &data ) )
        {
            print_available_values(eap_type);
            console_printf_terminal("EAP Type = %d\n", data );
        }
*/
    print_available_values(eap_type);
    }

}


void cmd_privacy_encrypt(ConParm_t parm[], U16 nParms)
{
   OS_802_11_ENCRYPTION_TYPES data;
   if( nParms )
    {
        TI_SetEncryptionType( g_id_adapter, (OS_802_11_ENCRYPTION_TYPES) parm[0].value );
    }
    else
    {
        print_available_values(encrypt_type);
        console_printf_terminal("Encryption=%d\n", !TI_GetEncryptionType( g_id_adapter, &data ) ? data : -1 );
    }

}
void cmd_privacy_credent(ConParm_t parm[], U16 nParms)
{

    if( nParms == 2 )
    {
        TI_SetCredentials(g_id_adapter,(tiCHAR *) parm[0].value, (tiCHAR *) parm[1].value);
    }
    else if( nParms == 1 )
        TI_SetCredentials(g_id_adapter,(tiCHAR *) parm[0].value, NULL);
    else
        return;

}

void cmd_privacy_PSKPassphrase(ConParm_t parm[], U16 nParms)
{
    char buf[PSK_BUFF_LEN], *pPassphrase;
    unsigned int len, is_hex_key = 0;


	if( nParms == 0 )
        return;

	len = strlen((char*)(parm[0].value));

	pPassphrase = (char*)(parm[0].value);

	memset(buf,0,PSK_BUFF_LEN);

    if( nParms >= 2 )
    {
#ifdef _WINDOWS
#else
        if( !stricmp((char *) parm[1].value, "hex") )
            is_hex_key = 1;
        else if(!stricmp((char *) parm[1].value, "text"))
            is_hex_key = 0;
#endif
    }

    if( is_hex_key )
    {
		if( len != PSK_HEXA_LENGTH )
        {
            console_printf_terminal("The hexa PSKPassphrase must be at length of %d hexa digits \n",PSK_HEXA_LENGTH);
            return ;
        }
    }
	else
	{
		if (len > MAX_PSK_STRING_LENGTH || len < MIN_PSK_STRING_LENGTH)
		{
            console_printf_terminal("The PSKPassphrase must be between %d to  %d chars \n", MIN_PSK_STRING_LENGTH, MAX_PSK_STRING_LENGTH);
            return ;
        }
	}

	memcpy(buf, (char*)(parm[0].value), len);

	/*TI_SetPSKPassPhrase*/
	TI_SetPSK(g_id_adapter, (tiCHAR *)buf);
}

void cmd_privacy_certificate(ConParm_t parm[], U16 nParms)
{
#ifdef _WINDOWS // TRS:HLC
#else
    console_printf_terminal("Set sertificate file : %s\n", (char*)parm[0].value);
    if(nParms == 1 )
        TI_SetCertificateParameters(g_id_adapter, (void*)parm[0].value, 0);
    else if(nParms == 2 )
        TI_SetCertificateParameters(g_id_adapter, (void*)parm[0].value,
                                                   (unsigned int)parm[1].value);
    else return;
#endif
//TRS end
}

void cmd_privacy_wpa_options(ConParm_t parm[], U16 nParms)
{
    if( nParms )
    {
        TI_SetWpaOptions(g_id_adapter, parm[0].value );
    }
    else
    {
        tiUINT32 data;
        static named_value_t wpa_options[] = {
            { OS_802_11_OPTION_ENABLE_PROMOTE_MODE,   "PROMOTE_MODE" },
            { OS_802_11_OPTION_ENABLE_PROMOTE_CIPHER, "PROMOTE_CIPHER"   },
            { OS_802_11_OPTION_ENABLE_ALL,            "All" }
        };

        print_available_values(wpa_options);
        if( !TI_GetWpaOptions(g_id_adapter, &data ) )
            console_printf_terminal("WPA option=%d\n", data );
    }
}

void cmd_privacy_getdefaultkey(ConParm_t parm[], U16 nParms)
{
    tiUINT32 DefaultKeyId;
    if (OK == TI_GetDefaultWepKey(g_id_adapter, &DefaultKeyId))
        console_printf_terminal("WEP default key ID = %d\n", DefaultKeyId );

}
unsigned int char_2_hexa( char c )
{
    if( c >= '0' && c <= '9' )
        return c - '0';
    else if( tolower(c) >= 'a' && tolower(c) <= 'f' )
        return tolower(c) - 'a' + 0x0a;
    console_printf_terminal("invalid symbol '%c'\n", c );
    return (unsigned int) -1;
}

void cmd_privacy_addkey(ConParm_t parm[], U16 nParms)
{
    OS_802_11_WEP data;
    char *buf;
    unsigned int i, len, is_hex_key = 1;
    U32 val, val_l;
    unsigned int key_id = 0;
    unsigned int def_flag = 0;

    buf = (char *) parm[0].value;

    key_id = (unsigned int)parm[1].value;

    if( parm[2].value )
        def_flag = 0x80000000;

    if( nParms >= 4 )
    {
#ifdef _WINDOWS
#else
        if( !stricmp((char *) parm[3].value, "hex") )
            is_hex_key = 1;
        else if(!stricmp((char *) parm[3].value, "text"))
            is_hex_key = 0;
#endif
    }

    memset(data.KeyMaterial,0,sizeof(data.KeyMaterial));

    len = strlen(buf);

    if( is_hex_key )
    {
        if( len % 2 )
        {
            console_printf_terminal("The hexa key should be even length\n");
            return ;
        }
        if(len <= 10) /*10 is number of character for key length 40 bit*/
            data.KeyLength = 5;
        else if(len <= 26) /*26 is number of character for key length 128 bit*/
            data.KeyLength = 13;
        else if(len <= 58) /*58 is number of character for key length 256 bit*/
            data.KeyLength = 29;
        else {
                console_printf_terminal("**Error key length\n" );
                return;
        }

        for( i=0; *buf && i < data.KeyLength; i++ )
        {
                val = char_2_hexa(*buf);
                if( val == (U32) -1 )
                    return;

                val_l = char_2_hexa(*(++buf));
                if( val_l == (U32) -1 )
                    return;

                data.KeyMaterial[i] = (tiUINT8)((val << 4) | val_l);
                buf++;
        }
    }
    else        /* for ascii key */
    {
        if(len <= 5) /*10 is number of character for key length 40 bit*/
            data.KeyLength = 5;
        else if(len <= 13) /*26 is number of character for key length 128 bit*/
            data.KeyLength = 13;
        else if(len <= 29) /*58 is number of character for key length 256 bit*/
            data.KeyLength = 29;
        else {
                console_printf_terminal("**Error key length\n" );
                return;
        }
        memcpy(data.KeyMaterial, buf, len );
    }

    data.KeyIndex = def_flag | key_id;
    data.Length = sizeof(OS_802_11_WEP);

#ifdef DEBUG_MESSAGES
    console_printf_terminal("cmd_privacy_addkey len = %d, type: %s\nkey:", data.KeyLength, is_hex_key ? "hex" : "text");
    for(i=0; i<SIZE_ARR(data.KeyMaterial); i++ )
        console_printf_terminal("%02x", (U32) data.KeyMaterial[i]);
    console_printf_terminal("\n");
#endif /*DEBUG_MESSAGES */
    TI_AddWEPKey(g_id_adapter, &data);
}


void cmd_privacy_removekey(ConParm_t parm[], U16 nParms)
{
    TI_RemoveWEPKey(g_id_adapter, (U32) parm[0].value );
}

void cmd_privacy_key_type(ConParm_t parm[], U16 nParms)
{

   if( nParms )
        {
        TI_SetKeyType( g_id_adapter, (OS_802_11_KEY_TYPES)parm[0].value );
        console_printf_terminal("CLI-: KeyType - %x\n",(tiUINT32)parm[0].value);
        }
    else
    {
        static named_value_t key_type[] = {
            { OS_KEY_TYPE_STATIC,             "STATIC" },
            { OS_KEY_TYPE_DYNAMIC,            "DYNAMIC"}
        };

        print_available_values(key_type);
    }

}
void cmd_privacy_mixed_mode(ConParm_t parm[], U16 nParms)
{
    tiBOOL data;

    if( nParms == 0 )
    {
       console_printf_terminal("Mixed Mode: 0 - FALSE, 1 - TRUE\n");
       data = FALSE;
       if( !TI_GetMixedMode(g_id_adapter, &data ) );
            console_printf_terminal("Mixed Mode =%s\n", data ? "True" : "False" );

    }
    else    /* param <read-only!!> */
        TI_SetMixedMode(g_id_adapter, (BOOL) parm[0].value);
}


/************** Roaming Manager functions  ******************/
void cmd_Roaming_enable(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    roamingMngrConfigParams.roamingMngrConfig.enableDisable = ROAMING_ENABLED;
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Roaming is enabled \n");
}


void cmd_Roaming_disable(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    roamingMngrConfigParams.roamingMngrConfig.enableDisable = ROAMING_DISABLED;
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Roaming is disabled \n");
}
void cmd_Roaming_lowPassFilter(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrConfig.lowPassFilterRoamingAttempt = (UINT16) parm[0].value;
    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Time in sec to wait before low quality Roaming Triggers, \n lowPassFilterRoamingAttempt = %d sec\n",
           roamingMngrConfigParams.roamingMngrConfig.lowPassFilterRoamingAttempt);
}

void cmd_Roaming_qualityIndicator(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrConfig.apQualityThreshold = (S8) parm[0].value;
    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Quality indicator (RSSI) to be used when comparing AP List matching quality, \n apQualityThreshold = %d \n",
           (roamingMngrConfigParams.roamingMngrConfig.apQualityThreshold));
}


void cmd_Roaming_dataRetryThreshold(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.dataRetryThreshold = (UINT8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("dataRetryThreshold = %d \n",
           roamingMngrConfigParams.roamingMngrThresholdsConfig.dataRetryThreshold);

}
void cmd_Roaming_numExpectedTbttForBSSLoss(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.numExpectedTbttForBSSLoss =  (UINT8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Number of expected TBTTs for BSS Loss event, \n numExpectedTbttForBSSLoss = %d \n",
           roamingMngrConfigParams.roamingMngrThresholdsConfig.numExpectedTbttForBSSLoss);

}
void cmd_Roaming_txRateThreshold(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.txRateThreshold =  (UINT8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("txRateThreshold = %d \n",
           roamingMngrConfigParams.roamingMngrThresholdsConfig.txRateThreshold);

}
void cmd_Roaming_lowRssiThreshold(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.lowRssiThreshold =  (S8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("lowRssiThreshold = %d \n",
           (roamingMngrConfigParams.roamingMngrThresholdsConfig.lowRssiThreshold));

}
void cmd_Roaming_lowSnrThreshold(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.lowSnrThreshold =  (S8)parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("lowSnrThreshold = %d \n", roamingMngrConfigParams.roamingMngrThresholdsConfig.lowSnrThreshold);
}
void cmd_Roaming_lowQualityForBackgroungScanCondition(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.lowQualityForBackgroungScanCondition = (S8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Indicator used to increase the background scan period when quality is low, \n lowQualityForBackgroungScanCondition = %d \n",
           (roamingMngrConfigParams.roamingMngrThresholdsConfig.lowQualityForBackgroungScanCondition));

}
void cmd_Roaming_normalQualityForBackgroungScanCondition(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.normalQualityForBackgroungScanCondition = (S8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Indicator used to reduce the background scan period when quality is normal, \n normalQualityForBackgroungScanCondition = %d \n",
           (roamingMngrConfigParams.roamingMngrThresholdsConfig.normalQualityForBackgroungScanCondition));

}

void cmd_Roaming_rssiFilterWeight(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.rssiFilterWeight =  (UINT8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Rssi Filter Weight = %d \n",
           (roamingMngrConfigParams.roamingMngrThresholdsConfig.rssiFilterWeight));
}

void cmd_Roaming_snrFilterWeight(ConParm_t parm[], U16 nParms)
{
    TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    if( nParms != 0 )
    {
        roamingMngrConfigParams.roamingMngrThresholdsConfig.snrFilterWeight =  (UINT8) parm[0].value;

    }
    TI_SetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));
    console_printf_terminal("Snr FIlter Weight = %d \n",
           (roamingMngrConfigParams.roamingMngrThresholdsConfig.snrFilterWeight));
}

void cmd_Roaming_getConfParams(ConParm_t parm[], U16 nParms)
{
   TI_GetRoamingConfiguration (g_id_adapter, (UINT8*)&roamingMngrConfigParams, sizeof(roamingMngrConfigParams_t));

   console_printf_terminal("Roaming is: %s \n", roamingMngrConfigParams.roamingMngrConfig.enableDisable ? "Enabled" : "Disabled");
   console_printf_terminal("lowPassFilterRoamingAttempt = %d sec, apQualityThreshold = %d\n",
          roamingMngrConfigParams.roamingMngrConfig.lowPassFilterRoamingAttempt,
		  roamingMngrConfigParams.roamingMngrConfig.apQualityThreshold);
   console_printf_terminal("Roaming Triggers' thresholds are: \n");
   console_printf_terminal("dataRetryThreshold = %d, lowQualityForBackgroungScanCondition = %d, \
lowRssiThreshold = %d, lowSnrThreshold = %d, normalQualityForBackgroungScanCondition = %d, \
numExpectedTbttForBSSLoss = %d, txRateThreshold = %d \n",
          roamingMngrConfigParams.roamingMngrThresholdsConfig.dataRetryThreshold,
		  roamingMngrConfigParams.roamingMngrThresholdsConfig.lowQualityForBackgroungScanCondition,
		  roamingMngrConfigParams.roamingMngrThresholdsConfig.lowRssiThreshold,
          roamingMngrConfigParams.roamingMngrThresholdsConfig.lowSnrThreshold,
		  roamingMngrConfigParams.roamingMngrThresholdsConfig.normalQualityForBackgroungScanCondition,
          roamingMngrConfigParams.roamingMngrThresholdsConfig.numExpectedTbttForBSSLoss,
          roamingMngrConfigParams.roamingMngrThresholdsConfig.txRateThreshold);
   console_printf_terminal("RSSI / SNR filter weights are: \n");
   console_printf_terminal("RSSI filter weight = %d, SNR filter weight = %d\n ",
   		roamingMngrConfigParams.roamingMngrThresholdsConfig.rssiFilterWeight,
   		roamingMngrConfigParams.roamingMngrThresholdsConfig.snrFilterWeight);
}


void cmd_bt_coe_enable(ConParm_t parm[], U16 nParms)
{
    if( nParms == 0 )
    {
        console_printf_terminal("Please enter enable value:\n");
        console_printf_terminal("0 - Enable\n");
        console_printf_terminal("1 - Disable\n");
        console_printf_terminal("2 - Auto\n");
    }
    else
    {
        TI_SetBtCoeEnable(g_id_adapter, parm[0].value);
    }
}


void cmd_bt_coe_rate(ConParm_t parm[], U16 nParms)
{
    U8 Values[NUM_OF_RATES_IN_SG];
    U8 Index;
    if( nParms != NUM_OF_RATES_IN_SG )
    {
 		console_printf_terminal("0 - 1Mbps (not recommended)\n");
		console_printf_terminal("1 - 2Mbps (not recommended)\n");
		console_printf_terminal("2 - 5.5Mbps\n");
		console_printf_terminal("3 - 6Mbps\n");
		console_printf_terminal("4 - 9Mbps\n");
		console_printf_terminal("5 - 11Mbps\n");
		console_printf_terminal("6 - 12Mbps\n");
		console_printf_terminal("7 - 18Mbps\n");
		console_printf_terminal("8 - 22Mbps (not in use)\n");
		console_printf_terminal("9 - 24Mbps\n");
		console_printf_terminal("10 - 36Mbps\n");
		console_printf_terminal("11 - 48Mbps\n");
		console_printf_terminal("12 - 54Mbps\n");
    }
    else
    {
        for (Index = 0; Index < NUM_OF_RATES_IN_SG; Index++ )
        {
			Values[Index] = (U8)parm[Index].value;  //TRS:MEB use cast to fix compile warning
        }
        TI_SetBtCoeRate(g_id_adapter, Values);
    }
}


void cmd_bt_coe_config(ConParm_t parm[], U16 nParms)
{
    U32 Values[NUM_OF_CONFIG_PARAMS_IN_SG];
    U8 Index = 0;

    if( nParms != NUM_OF_CONFIG_PARAMS_IN_SG )
    {
        console_printf_terminal("Please enter valid config values:\n");

		console_printf_terminal("Param %d - wlanRxMinRateToRespectBtHp (0 - all,2,5,6,9,11,12,18,22,24,36,48,54)  \n",Index++);
        console_printf_terminal("Param %d - btHpMaxTime (100 - 15000)\n",Index++);
        console_printf_terminal("Param %d - wlanHpMaxTime (100 - 15000)\n",Index++);
        console_printf_terminal("Param %d - senseDisableTimer (100 - 15000)\n",Index++);
        console_printf_terminal("Param %d - protectiveRxTimeBeforeBtHp (10 - 2300)\n",Index++);
        console_printf_terminal("Param %d - protectiveTxTimeBeforeBtHp (10 - 2300)\n",Index++);
        console_printf_terminal("Param %d - protectiveRxTimeBeforeBtHpFastAp (10 - 20000)\n",Index++);
        console_printf_terminal("Param %d - protectiveTxTimeBeforeBtHpFastAp (10 - 20000)\n",Index++);
        console_printf_terminal("Param %d - protectiveWlanCycleTimeForFastAp (2000 - 65535)\n",Index++);
        console_printf_terminal("Param %d - btAntiStarvationPeriod (0 - 15000) \n",Index++);
        console_printf_terminal("Param %d - timeoutNextBtLpPacket (400 - 10000)\n",Index++);
		console_printf_terminal("Param %d - wakeUpTimeBeforeBeacon   (0 - 20000)  \n",Index++);
		console_printf_terminal("Param %d - hpdmMaxGuardTime   (0 - 50000)  \n",Index++);
		console_printf_terminal("Param %d - timeoutNextWlanPacket   (100 - 50000)  \n",Index++);
        console_printf_terminal("Param %d - sgAntennaType (0 - Single | 1 - Dual | 2 - Single+ )\n",Index++);
        console_printf_terminal("Param %d - signalingType (0 - Legacy | 1 - Palau | 2 - Other)\n",Index++);
        console_printf_terminal("Param %d - afhLeverageOn (0 - OFF | 1 - GPIO  | 2 - ON)\n",Index++);
        console_printf_terminal("Param %d - numberQuietCycle (0 - 10)\n",Index++);
        console_printf_terminal("Param %d - maxNumCts (0 - 10)\n",Index++);
        console_printf_terminal("Param %d - numberOfWlanPackets (1 - 10)\n",Index++);
        console_printf_terminal("Param %d - numberOfBtPackets (2 - 10)\n",Index++);
        console_printf_terminal("Param %d - numberOfMissedRxForAvalancheTrigger (1 - 255)\n",Index++);
        console_printf_terminal("Param %d - wlanElpHpSupport (0 - 1)\n",Index++);
        console_printf_terminal("Param %d - btAntiStarvationNumberOfCyclesWithinThePeriod (0 - 15)  \n",Index++);
        console_printf_terminal("Param %d - ackModeDuringBtLpInDualAnt (0 - 1)  \n",Index++);
        console_printf_terminal("Param %d - allowPaSdToggleDuringBtActivityEnable (0 - 1)  \n",Index++);
		console_printf_terminal("Param %d - sgAutoModeNoCts   (0 - 1)  \n",Index++);
		console_printf_terminal("Param %d - numOfBtHpRespectedReq   (0 - 20)  \n",Index++);
   }
   else
    {
        for (Index = 0; Index < NUM_OF_CONFIG_PARAMS_IN_SG; Index++ )
        {
            Values[Index] = parm[Index].value;
        }

		if ( ( (is_value_rate(Values[0])) && (Values[0] != 1) ) || (Values[0] == 0) )
		{
			TI_SetBtCoeConfig(g_id_adapter, (tiUINT32 *)Values);
		}
		else
		{
			console_printf_terminal("Error: Param 26 - wlanRxMinRateToRespectBtHp (0 - all,2,5,6,9,11,12,18,22,24,36,48,54)  \n");
		}
    }
}


void cmd_bt_coe_get_status(ConParm_t parm[], U16 nParms)
{
    U32 Values[NUM_OF_STATUS_PARAMS_IN_SG];
    /* The print is done inside the module */
    console_printf_terminal("Done by driver - ");        
    if( TI_SetBtCoeGetStatus(g_id_adapter,(tiUINT32 *) Values) == OK ) {
        console_printf_terminal("Ok\n");
/*      console_printf_terminal("BT Coxistence status: \n\n");
        console_printf_terminal("Enable: %d\n", Values[0]);
        console_printf_terminal("Rate: %d\n", Values[1]);
        console_printf_terminal("BtSignaling: %d\n", Values[2]);
        console_printf_terminal("BtHPMaxTime: %d\n", Values[3]);
        console_printf_terminal("WlanHPMaxTime: %d\n", Values[4]);
        console_printf_terminal("WlanEOSMaxPacket: %d\n", Values[5]);
        console_printf_terminal("WlanEOSMaxPacketTimeOut: %d\n", Values[6]);
        console_printf_terminal("BtPTAMaxPacket: %d\n", Values[7]);
        console_printf_terminal("BtPTAMaxPacketTimeOut: %d\n", Values[8]);
        console_printf_terminal("WlanSlowAPSocial: %d\n", Values[9]);
        console_printf_terminal("WlanSlowAPMaxCTS: %d\n", Values[10]);
        console_printf_terminal("WlanSlowAPMaxTimeToCTS: %d\n", Values[11]);
        console_printf_terminal("T8_temporary: %d\n", Values[12]);
        console_printf_terminal("BTtoWLANSwitchTime: %d\n", Values[13]); */
    }
    else
    {
        console_printf_terminal("Fail\n");    
/*      console_printf_terminal("Error reading status!\n"); */
    }
}


void cmd_PLT_RxPerStart(ConParm_t parm[], U16 nParms)
{
	UINT32 Status = TI_PLT_RxPerStart(g_id_adapter);
	if (Status == OK)
		console_printf_terminal("Plt RX counters started\n");
	else
		console_printf_terminal("Plt RX counters start failed\n");
}

void cmd_PLT_RxPerStop(ConParm_t parm[], U16 nParms)
{
	UINT32 Status = TI_PLT_RxPerStop(g_id_adapter);
	if (Status == OK)
		console_printf_terminal("Plt RX counters stoped\n");
	else
		console_printf_terminal("Plt RX counters stop failed\n");
}

void cmd_PLT_RxPerClear(ConParm_t parm[], U16 nParms)
{
	UINT32 Status = TI_PLT_RxPerClear(g_id_adapter);
	if (Status == OK)
		console_printf_terminal("Plt RX counters cleard\n");
	else
		console_printf_terminal("Plt RX counters clear failed\n");
}

void cmd_PLT_RxPerGet(ConParm_t parm[], U16 nParms)
{
	PltRxPer_t PltRxPer;
	UINT32 Status = TI_PLT_RxPerGetResults(g_id_adapter, &PltRxPer);

	if (Status == OK)
	{
		console_printf_terminal("FCSErrorCount = %d\n", PltRxPer.FCSErrorCount);
		console_printf_terminal("PLCPErrorCount  = %d\n", PltRxPer.PLCPErrorCount);
        console_printf_terminal("SeqNumMissCount = %d\n", PltRxPer.SeqNumMissCount);
		console_printf_terminal("TotalFrameCount = %d\n", PltRxPer.TotalFrameCount);
	}
	else
		console_printf_terminal("Plt RX counters Get results failed\n");
}

void cmd_PLT_RegisterRead(ConParm_t parm[], U16 nParms)
{
    tiUINT32 RegAddress;
    tiUINT32 RegValue;
    tiUINT32 Status;
    char* pTmp;

    /* Converting hex string to tiUINT32*/
    pTmp = (char*)parm[0].value;
    sscanf(pTmp , "%x", &RegAddress);

    /*Call the API function */
    Status = TI_PLT_ReadRegister(g_id_adapter, RegAddress, &RegValue);
    if( Status == OK )
        console_printf_terminal("Reg. %#lx = %#x (%d)\n", RegAddress, RegValue, RegValue );
}

void cmd_PLT_RegisterWrite(ConParm_t parm[], U16 nParms)
{
    tiUINT32 RegAddress;
    tiUINT32 RegValue = 0;
    tiUINT32 Status;
    char* pTmp;

    /* Converting hex string to tiUINT32*/
    printf("cmd_PLT_RegisterWrite\n");
    pTmp = (char*)parm[0].value;
    sscanf(pTmp, "%x", &RegAddress);
    pTmp = (char*)parm[1].value;
    sscanf(pTmp , "%x", &RegValue);
    printf("cmd_PLT_RegisterWrite %x %x\n", RegAddress, RegValue);

    /*Call the API function */
    Status = TI_PLT_WriteRegister(g_id_adapter, RegAddress, RegValue );
    if (Status == OK)
        console_printf_terminal("Plt register 0x%x is set to 0x%x OK.\n", RegAddress, RegValue);
    else
        console_printf_terminal("Plt register 0x%x is set to 0x%x NOK.\n", RegAddress, RegValue);

}


void cmd_PLT_TxContinues(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   PltTxContinues_t PltTxContinues;
   if ((nParms == 0) || (nParms > 10))
   {
      console_printf_terminal("\n");
	  console_printf_terminal("Param 0 - Band (0 - 2.4Ghz(B/G), 1 - 5Ghz(A), 2 - Japan(4.9Ghz))\n");
	  console_printf_terminal("Param 1 - Channel: (1..14(2.4Ghz), 1..180(5Ghz)) \n");
	  console_printf_terminal("Param 2 - Rate: \n");
      console_printf_terminal("                1  - 1Mbps\n");
      console_printf_terminal("                2  - 2Mbps\n");
      console_printf_terminal("                3  - 5.5Mbps\n");
      console_printf_terminal("                4  - 11Mbps\n");
      console_printf_terminal("                6  - 6Mbps\n");
      console_printf_terminal("                7  - 9Mbps\n");
      console_printf_terminal("                8  - 12Mbps\n");
      console_printf_terminal("                9  - 18Mbps\n");
      console_printf_terminal("                10 - 24Mbps\n");
      console_printf_terminal("                11 - 36Mbps\n");
      console_printf_terminal("                12 - 48Mbps\n");
      console_printf_terminal("                13 - 54Mbps \n");
	  console_printf_terminal("Param 3 - preamble (0-long, 1-short)\n");
	  console_printf_terminal("Param 4 - Delay between packets (uSec)\n");
      console_printf_terminal("Param 5 - Number of TX frames (0 - endless)\n");
      console_printf_terminal("Param 6 - Test mode (5-Random data, 9-ZOZO(0,1,0,1,...))\n");
      console_printf_terminal("Param 7 - Sequance number mode(0 - fixed, 1 - incremented)\n");
      console_printf_terminal("Param 8 - packet Data legth [bytes] (0 - 2284)\n");
      console_printf_terminal("Param 9 - peer mac address: [xx:xx:xx:xx:xx:xx]\n");

   }
   else
   {
	   PltTxContinues.band 	            = (UINT8) parm[0].value;
	   PltTxContinues.chID   	        = parm[1].value;
	   PltTxContinues.rate   	        = parm[2].value;
	   PltTxContinues.preamble	        = (UINT8) parm[3].value;
	   PltTxContinues.InterPacketDelay  = parm[4].value;
       PltTxContinues.NumOfFrames       = parm[5].value;
	   PltTxContinues.mode              = (UINT8) parm[6].value;
       PltTxContinues.aSeqNumMode       = parm[7].value;
       PltTxContinues.aPacketLength     = parm[8].value;
       hexStr2MACAddr( (char*)parm[9].value, &(PltTxContinues.aPeerMacAddr) );


	   Status = TI_PLT_TxContiues(g_id_adapter, &PltTxContinues);
		if (Status == OK)
			console_printf_terminal("OK\n");
		else
			console_printf_terminal("NOK\n");

   }
}

void cmd_PLT_TxCW(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   TestCmdChannelBand_t PltTxCW;

   if ((nParms == 0) || (nParms > 2))
   {
	  console_printf_terminal("Param 0 - Band (0 - 2.4Ghz(B/G), 1 - 5Ghz(A), 2 - Japan(4.9Ghz))\n");
	  console_printf_terminal("Param 1 - Channel(1..14(2.4Ghz), 1..180(5Ghz))\n");
   }
   else
   {
	   PltTxCW.band 	= (RadioBand_e) parm[0].value;
	   PltTxCW.channel 	= (Channel_e) parm[1].value;

	   Status = TI_PLT_TxCW(g_id_adapter, &PltTxCW);
		if (Status == OK)
			console_printf_terminal("OK\n");
		else
			console_printf_terminal("NOK\n");

   }
}

void cmd_PLT_TxStop(ConParm_t parm[], U16 nParms)
{
	UINT32 Status = TI_PLT_TxStop(g_id_adapter);
	if (Status == OK)
		console_printf_terminal("OK\n");
	else
		console_printf_terminal("NOK\n");
}

void cmd_PLT_MIB_CounterTable(ConParm_t parm[], U16 nParms)
{
    PLT_MIB_t Mib;
    UINT32 Status;
    memset(&Mib, 0, sizeof(Mib));
    Mib.aMib = PLT_MIB_countersTable;

    Status = TI_PLT_ReadMIB(g_id_adapter, &Mib);
    if (Status == OK)
    {
       console_printf_terminal("FCS error count= %d \nPLCP  error count = %d\n",
							   Mib.aData.CounterTable.FCSErrorCount,
							   Mib.aData.CounterTable.PLCPErrorCount);
    }
    else
    {
		console_printf_terminal("NOK\n");
    }
}

void cmd_PLT_MIB_StationID(ConParm_t parm[], U16 nParms)
{
    PLT_MIB_t Mib;
    UINT32 Status;
    memset(&Mib, 0, sizeof(Mib));
    Mib.aMib = PLT_MIB_dot11StationId;

    Status = TI_PLT_ReadMIB(g_id_adapter, &Mib);
    if (Status == OK)
    {
       console_printf_terminal("MIB_StationID = %02x:%02x:%02x:%02x:%02x:%02x\n",
                                Mib.aData.StationId.addr[5],
                                Mib.aData.StationId.addr[4],
                                Mib.aData.StationId.addr[3],
                                Mib.aData.StationId.addr[2],
                                Mib.aData.StationId.addr[1],
                                Mib.aData.StationId.addr[0]);
    }
    else
    {
		console_printf_terminal("NOK\n");
    }


}


void cmd_modify_ctsToSelf(ConParm_t parm[], U16 nParms)
{
    PLT_MIB_t Mib;
    UINT32 Status;
    memset(&Mib, 0, sizeof(Mib));
    Mib.aMib = PLT_MIB_ctsToSelf;

	if (nParms != 1)
	{
	   console_printf_terminal("CTS to self: [0 - Disable, 1 - Enable]\n");
	}

	if (nParms == 0) /*Get ctsToSelf */
    {
        Status = TI_PLT_ReadMIB(g_id_adapter, &Mib);
        if (OK == Status)
            console_printf_terminal("ctsToSelf = %s (%d)\n",
            Mib.aData.CTSToSelfEnable?"Enable":"Disable",
            Mib.aData.CTSToSelfEnable);
    }
    else if (nParms == 1)
    {
        Mib.Length = sizeof(Mib.aData.CTSToSelfEnable);
        Mib.aData.CTSToSelfEnable = parm[0].value;
        if (OK != TI_PLT_WriteMIB(g_id_adapter, &Mib))
            console_printf_terminal("TI_PLT_WriteMIB failed\n");
    }
}

void cmd_get_arpIpTable(ConParm_t parm[], U16 nParms)
{
    PLT_MIB_t Mib;
    UINT32 Status;
    memset(&Mib, 0, sizeof(Mib));
    Mib.aMib = PLT_MIB_arpIpAddressesTable;
    Status = TI_PLT_ReadMIB(g_id_adapter, &Mib);
    if (OK == Status)
    {
       int FilteringEnable = Mib.aData.ArpIpAddressesTable.FilteringEnable;
       console_printf_terminal("ARP IP Table:\n");
       console_printf_terminal("FilteringEnable: %s (%d)\n",
           FilteringEnable?"Enable":"Disable",
           FilteringEnable);
       console_printf_terminal("ArpIpAddress: %d.%d.%d.%d\n",
           Mib.aData.ArpIpAddressesTable.addr[0],
           Mib.aData.ArpIpAddressesTable.addr[1],
           Mib.aData.ArpIpAddressesTable.addr[2],
           Mib.aData.ArpIpAddressesTable.addr[3]
           );
    }
}

void cmd_get_GroupAddressTable(ConParm_t parm[], U16 nParms)
{
    PLT_MIB_t Mib;
    UINT32 Status;
    memset(&Mib, 0, sizeof(Mib));
    Mib.aMib = PLT_MIB_dot11GroupAddressesTable;
    Status = TI_PLT_ReadMIB(g_id_adapter, &Mib);
    if (OK == Status)
    {
       int FilteringEnable = Mib.aData.GroupAddressTable.bFilteringEnable;
       int i;

       console_printf_terminal("Group addresses Table:\n");
       console_printf_terminal("FilteringEnable: %s (%d)\n",
           FilteringEnable?"Enable":"Disable",
           FilteringEnable);
       console_printf_terminal("nNumberOfAddresses: %d\n", Mib.aData.GroupAddressTable.nNumberOfAddresses);
       console_printf_terminal("Group addresses: \n");

       for (i=0; i<Mib.aData.GroupAddressTable.nNumberOfAddresses; i++)
       console_printf_terminal("%x:%x:%x:%x:%x:%x\n",
           Mib.aData.GroupAddressTable.GroupTable[i].addr[0],
           Mib.aData.GroupAddressTable.GroupTable[i].addr[1],
           Mib.aData.GroupAddressTable.GroupTable[i].addr[2],
           Mib.aData.GroupAddressTable.GroupTable[i].addr[3],
           Mib.aData.GroupAddressTable.GroupTable[i].addr[4],
           Mib.aData.GroupAddressTable.GroupTable[i].addr[5]
           );
    }
}

void cmd_PLT_TxCalGainGet(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   PltGainGet_t PLTGainGet;

   Status = TI_PLT_TxCalGainGet(g_id_adapter, &PLTGainGet);
   if (Status == OK)
   {
       console_printf_terminal("\n");
       console_printf_terminal("TxGain: %d(0x%x)\n", PLTGainGet.TxGain, PLTGainGet.TxGain);
       console_printf_terminal("TxUpperBound: %d(0x%x)\n", PLTGainGet.TxUpperBound, PLTGainGet.TxUpperBound);
       console_printf_terminal("TxLowerBound: %d(0x%x)\n", PLTGainGet.TxLowerBound, PLTGainGet.TxLowerBound);
   }
   else
       console_printf_terminal("NOK\n");
}


void cmd_PLT_TxCalGainAdjust(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   UINT32 GainChange;
   OS_802_11_POWER_LEVELS mode;


   if ((nParms == 0) || (nParms > 1))
   {
	   console_printf_terminal("\n");
       console_printf_terminal("Param 0 - Gain change (db)\n");
   }
   else
   {
       GainChange = parm[0].value;

        /* Check Power mode (works only in "awake" mode !!!) */
        TI_GetPowerLevelDefault(g_id_adapter, &mode );
        if(mode == OS_POWER_LEVEL_AWAKE)
        {
	   Status = TI_PLT_TxCalGainAdjust(g_id_adapter, GainChange);
		    if (Status == OK)
			    console_printf_terminal("OK\n");
		    else
			    console_printf_terminal("NOK\n");
        }
        else
        {
	        console_printf_terminal("Gain Adjust was not performed becouse Default power-mode is not AWAKE\n");
	        console_printf_terminal("Please change defaultPowerLevel parametr in tiwlan.ini file first\n");
        }
   }
}

void cmd_PLT_TxCalStart(ConParm_t parm[], U16 nParms)
{
       UINT32 Status;
	   PltTxCalibrationRequest_t tTxStart;

	   if (nParms != 1)
	   {
		   console_printf_terminal("\nParam 0 - Tx Power [0-255]\n");
	   }
	   else
	   {
        /* use U8 cast to fix compile warning */
		   tTxStart.refTxPower = (U8)parm[0].value;
		   Status = TI_PLT_TxCalStart(g_id_adapter,&tTxStart);
		   if (Status == OK)
			   console_printf_terminal("OK\n");
		   else
			   console_printf_terminal("NOK\n");
	   }
}

void cmd_PLT_TxCalStop(ConParm_t parm[], U16 nParms)
{
       UINT32 Status;

	   Status = TI_PLT_TxCalStop(g_id_adapter);
       if (Status == OK)
           console_printf_terminal("OK\n");
       else
           console_printf_terminal("NOK\n");
}

void cmd_PLT_RxTxCalNVSUpdateBuffer(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   PltNvsResultsBuffer_t PltNvsResultsBuffer;
   int i;

   memset(&PltNvsResultsBuffer, 0, sizeof(PltNvsResultsBuffer));
   Status = TI_PLT_RxTxCalNVSUpdateBuffer(g_id_adapter, &PltNvsResultsBuffer);
   if (Status == OK)
   {
       console_printf_terminal("NVSUpdateBuffer -  number of tables:%d\n", PltNvsResultsBuffer.numOfTables);
       console_printf_terminal("# \t Offset\t Size\t Data\n");
       console_printf_terminal("###################################################################\n");
       for (i=0; (i<PltNvsResultsBuffer.numOfTables) && (i<NVS_RESULTS_MAX_NUM_OF_TABLES); i++)
       {
        int j;
        console_printf_terminal("#%d\t %p\t %.4d\t ",
                                i,
                                PltNvsResultsBuffer.tables[i].offset,
                                PltNvsResultsBuffer.tables[i].size);
        for (j=0; (j<PltNvsResultsBuffer.tables[i].size) && (j<NVS_RESULTS_MAX_UPDATE_TABLE_SIZE); j++)
        {
            console_printf_terminal("%.2x ", PltNvsResultsBuffer.tables[i].data[j]);
        }
        console_printf_terminal("\n");
       }
   }
   else
       console_printf_terminal("NOK\n");
}

void cmd_PLT_RxCal(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   PltRxCalibrationRequest_t PltRxCalibrationRequest;
   OS_802_11_POWER_LEVELS mode;

   if ((nParms == 0) || (nParms > 5))
   {
      console_printf_terminal("\n");
	  console_printf_terminal("Param 0 - Expected Rssi (db)\n");
	  console_printf_terminal("Param 1 - Channel\n");
	  console_printf_terminal("Param 2 - Band (0 - 2.4Ghz(B/G), 1 - 5Ghz(A), 2 - Japan(4.9Ghz)\n");
      console_printf_terminal("Param 3 - Interval between samples(uSec) [100]\n");
      console_printf_terminal("Param 4 - Number of samples [1000]\n");
   }
   else
   {
	   PltRxCalibrationRequest.expectedRssi	= parm[0].value;
	   PltRxCalibrationRequest.channel = (UINT8)parm[1].value;
	   PltRxCalibrationRequest.band = (UINT8)parm[2].value;
       PltRxCalibrationRequest.intervalBetweenSamplesUsec = parm[3].value;
       PltRxCalibrationRequest.numOfSamples = (UINT16)parm[4].value;

        /* Check Power mode (works only in "awake" mode !!!) */
        TI_GetPowerLevelDefault(g_id_adapter, &mode );
        if(mode == OS_POWER_LEVEL_AWAKE)
        {
	        Status = TI_PLT_RxCal(g_id_adapter, &PltRxCalibrationRequest);
	        if (Status == OK)
		        console_printf_terminal("OK\n");
	        else
		        console_printf_terminal("NOK\n");
        }
        else
        {
	        console_printf_terminal("Rx calibration was not performed becouse Default power-mode is not AWAKE\n");
	        console_printf_terminal("Please change defaultPowerLevel parametr in tiwlan.ini file first\n");
        }
   }
}

void cmd_PLT_RadioTune(ConParm_t parm[], U16 nParms)
{
   UINT32 Status;
   TestCmdChannelBand_t ChannelBand;
   OS_802_11_POWER_LEVELS mode;


   if ((nParms == 0) || (nParms > 2))
   {
	  console_printf_terminal("Param 0 - Band (0-2.4Ghz, 1-5Ghz, 2-4.9Ghz)\n");
	  console_printf_terminal("Param 1 - Channel\n");
   }
   else
   {
	   ChannelBand.band 	= (RadioBand_e) parm[0].value;
	   ChannelBand.channel 	= (Channel_e) parm[1].value;

	    /* Check Power mode (works only in "awake" mode !!!) */
        TI_GetPowerLevelDefault(g_id_adapter, &mode );
		if(mode == OS_POWER_LEVEL_AWAKE)
		{
			Status = TI_PLT_RadioTune(g_id_adapter, &ChannelBand);
			if (Status == OK)
				console_printf_terminal("OK\n");
			else
				console_printf_terminal("NOK\n");
		}
		else
		{
			console_printf_terminal("Radio tune was not performed becouse Default power-mode is not AWAKE\n");
			console_printf_terminal("Please change defaultPowerLevel parametr in tiwlan.ini file first\n");
		}
	}
}


#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */
