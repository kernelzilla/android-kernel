/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/


#ifndef TI1610_IOCTL_UTILS
#define TI1610_IOCTL_UTILS

#include <linux/netdevice.h>
#include <asm/uaccess.h>            /* copy_from_user() */

#include "osTIType.h"

void print_memory_dump(char *addr, int size );

int util_get_statistics(struct net_device *dev, ULONG *statistic_data, ULONG *length );

int util_get_param( struct net_device *dev, UINT32 param_type, ULONG *data, UINT32 max_param_len);
int util_set_param( struct net_device *dev, UINT32 param_type, ULONG *data, UINT32 max_param_len );
int util_get_report_param( struct net_device *dev, UINT32 param_type, ULONG *data, UINT32 max_param_len);
int util_set_report_param( struct net_device *dev, UINT32 param_type, ULONG *data, UINT32 max_param_len);

/*int util_get_bssid(struct net_device *dev, int cmd, union iwreq_data *wrqu );*/
int util_get_network_type_in_use(struct net_device *dev, /*int cmd, */ULONG *data );
int util_write_hw_register(struct net_device *dev, ULONG *data );
int util_read_hw_register(struct net_device *dev, ULONG *data );
int util_hal_debug_print(struct net_device *dev, ULONG *data);
int util_get_sw_version(struct net_device *dev, ULONG *data );
int util_get_network_types_supported(struct net_device *dev, ULONG *data );
int util_get_number_of_antennas(struct net_device *dev, ULONG *data );
int util_get_rssi(struct net_device *dev, ULONG *data );
int util_get_bssid_list(struct net_device *dev, ULONG *data, ULONG *length );
int util_set_privacy_filter(struct net_device *dev, ULONG *data );
int util_get_privacy_filter(struct net_device *dev, ULONG *data, ULONG *length );
int util_disassociate(struct net_device *dev);
int util_set_ssid(struct net_device *dev, void *buf, ULONG length);		/* data - pointer to OS_802_11_SSID */
int util_get_ssid(struct net_device *dev, ULONG configMgr_param, void *data, ULONG *length);
/*int util_get_ssid(struct net_device *dev, void *data, ULONG *length);		( data - pointer to OS_802_11_SSID) */

int util_get_desired_channel(struct net_device *dev, ULONG *data);
int util_get_authentication_mode(struct net_device *dev, ULONG *data);
int util_get_rts_threshold(struct net_device *dev, ULONG *data);
int util_get_short_preamble(struct net_device *dev, ULONG *data);

int util_poll_ap_packets (struct net_device *dev);
int util_config_tx_classifier(struct net_device *dev,UINT8 *inParamsBuff,UINT32 inParamsBuffLen);


int util_get_fragmentation_threshold(struct net_device *dev, ULONG *data);
int util_get_infrastructure_mode(struct net_device *dev, ULONG *data);
int util_get_desired_infrastructure_mode(struct net_device *dev, ULONG *data);
int util_get_wep_status(struct net_device *dev, ULONG *data);
int util_get_power_mode(struct net_device *dev, ULONG *data);
int util_get_tx_antenna(struct net_device *dev, ULONG *data);
int util_get_rx_antenna(struct net_device *dev, ULONG *data);
int util_get_short_preamble(struct net_device *dev, ULONG *data);
int util_get_enable_leap(struct net_device *dev, ULONG *data);
int util_set_wep_status(struct net_device *dev, ULONG *data);
int util_add_wep(struct net_device *dev, void *data, ULONG length);
int util_add_key(struct net_device *dev, void *data, ULONG length);
int util_remove_key(struct net_device *dev, void *data, ULONG length);
int util_remove_wep(struct net_device *dev, void *data, ULONG length);
int util_get_association_info(struct net_device *dev, ULONG *data, ULONG *length );
int ti1610_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
#endif
