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

#ifndef __PARAM_IN_H__
#define __PARAM_IN_H__

#include "osTIType.h"
#include "commonTypes.h"



/* In this file are defined the parameter that are for internal use of the software only. */
/* Following are the parameters numbers. Each module can have 256 parameters */
typedef enum
{
    /* HAL Control section */
    HAL_CTRL_AID_PARAM                                  = HAL_CTRL_MODULE_PARAM | 0x01,
    HAL_CTRL_RSN_HW_ENC_DEC_ENABLE_PARAM                = HAL_CTRL_MODULE_PARAM | 0x02,  
    HAL_CTRL_RSN_KEY_ADD_PARAM                          = HAL_CTRL_MODULE_PARAM | 0x03, 
    HAL_CTRL_RSN_KEY_REMOVE_PARAM                       = HAL_CTRL_MODULE_PARAM | 0x04, 
    HAL_CTRL_RSN_DEFAULT_KEY_ID_PARAM                   = HAL_CTRL_MODULE_PARAM | 0x05, 
    HAL_CTRL_RSN_SECURITY_MODE_PARAM                	= HAL_CTRL_MODULE_PARAM | 0x06,  
    HAL_CTRL_RSN_SECURITY_ALARM_CB_SET_PARAM            = HAL_CTRL_MODULE_PARAM | 0x07,  
    HAL_CTRL_ACX_STATISTICS_PARAM                       = HAL_CTRL_MODULE_PARAM | 0x08,
    HAL_CTRL_MEDIUM_OCCUPANCY_PARAM                     = HAL_CTRL_MODULE_PARAM | 0x09,
    HAL_CTRL_DISABLE_POWER_MANAGEMENT_AUTO_CONFIG_PARAM = HAL_CTRL_MODULE_PARAM | 0x0a,
    HAL_CTRL_ENABLE_POWER_MANAGEMENT_AUTO_CONFIG_PARAM  = HAL_CTRL_MODULE_PARAM | 0x0b,
    HAL_CTRL_SG_ENABLE_PARAM                            = HAL_CTRL_MODULE_PARAM | 0x0c,
    HAL_CTRL_SG_CONFIG_PARAM                            = HAL_CTRL_MODULE_PARAM | 0x0d,
    

#ifdef EXC_MODULE_INCLUDED
    HAL_CTRL_RSN_EXC_SW_ENC_ENABLE_PARAM                = HAL_CTRL_MODULE_PARAM | 0x0e,  
    HAL_CTRL_RSN_EXC_MIC_FIELD_ENABLE_PARAM             = HAL_CTRL_MODULE_PARAM | 0x0f,  
#endif /* EXC_MODULE_INCLUDED*/
    HAL_CTRL_TX_OP_LIMIT                                = HAL_CTRL_MODULE_PARAM | 0x10,
    HAL_CTRL_NOISE_HISTOGRAM_PARAM                      = HAL_CTRL_MODULE_PARAM | 0x11,
    HAL_CTRL_TSF_DTIM_MIB                               = HAL_CTRL_MODULE_PARAM | 0x12,
    HAL_CTRL_REVISION                                   = HAL_CTRL_MODULE_PARAM | 0x13,
	HAL_CTRL_POWER_LEVEL_TABLE_PARAM					= HAL_CTRL_MODULE_PARAM | 0x14,
    /* Connection section */
    CONN_TYPE_PARAM                                     = CONN_MODULE_PARAM | 0x01,

    /* MLME section */
    MLME_LEGACY_TYPE_PARAM                              = MLME_SM_MODULE_PARAM | 0x01,
    MLME_RE_ASSOC_PARAM                                 = MLME_SM_MODULE_PARAM | 0x02,
    MLME_TNET_WAKE_ON_PARAM                             = MLME_SM_MODULE_PARAM | 0x03,
    MLME_CAPABILITY_PARAM                               = MLME_SM_MODULE_PARAM | 0x04,

    /* Auth section */
    AUTH_LEGACY_TYPE_PARAM                              = AUTH_MODULE_PARAM | 0x01,

    /* RX data section */
    RX_DATA_EAPOL_DESTINATION_PARAM                     = RX_DATA_MODULE_PARAM | 0x01,
    RX_DATA_PORT_STATUS_PARAM                           = RX_DATA_MODULE_PARAM | 0x02,

    /* TX data section */
    TX_DATA_CURRENT_PRIVACY_INVOKE_MODE_PARAM           = TX_DATA_MODULE_PARAM | 0x01,  
    TX_DATA_EAPOL_ENCRYPTION_STATUS_PARAM               = TX_DATA_MODULE_PARAM | 0x02,
    TX_DATA_HAL_INTERFACE_STATUS_PARAM          = TX_DATA_MODULE_PARAM | 0x03,          
    TX_DATA_802_11_POWER_SAVE_STATUS_PARAM              = TX_DATA_MODULE_PARAM | 0x04,          

    /* CTRL data section */
    CTRL_DATA_CURRENT_MODULATION_TYPE_PARAM             = CTRL_DATA_MODULE_PARAM | 0x01,
    CTRL_DATA_CURRENT_BASIC_RATE_PARAM                  = CTRL_DATA_MODULE_PARAM | 0x02,
    CTRL_DATA_CURRENT_BASIC_MODULATION_PARAM            = CTRL_DATA_MODULE_PARAM | 0x03,
    CTRL_DATA_CURRENT_BASIC_RATE_MASK_PARAM             = CTRL_DATA_MODULE_PARAM | 0x04,
    CTRL_DATA_CURRENT_ACTIVE_RATE_PARAM                 = CTRL_DATA_MODULE_PARAM | 0x05,


    /* SiteMgr section */   
    SITE_MGR_POWER_CONSTRAINT_PARAM                     = SITE_MGR_MODULE_PARAM | 0x01,
    SITE_MGR_BEACON_INTERVAL_PARAM                      = SITE_MGR_MODULE_PARAM | 0x02,
    SITE_MGR_RADIO_BAND_PARAM                           = SITE_MGR_MODULE_PARAM | 0x03,
    SITE_MGR_NEXT_DTIM_TIME_STAMP_PARAM                 = SITE_MGR_MODULE_PARAM | 0x04,
    SITE_MGR_SITE_CAPABILITY_PARAM                      = SITE_MGR_MODULE_PARAM | 0x05,
    SITE_MGR_4X_PARAM                                   = SITE_MGR_MODULE_PARAM | 0x06,
    SITE_MGR_RGSTRY_BASIC_RATE_SET_MASK                 = SITE_MGR_MODULE_PARAM | 0x08,
    SITE_MGR_BEACON_RECV                                = SITE_MGR_MODULE_PARAM | 0x09,
    SITE_MGR_DTIM_PERIOD_PARAM                          = SITE_MGR_MODULE_PARAM | 0x0A,


    /* Previous Primary Site */
    SITE_MGR_PREV_SITE_BSSID_PARAM                      = SITE_MGR_MODULE_PARAM | 0x0B,
    SITE_MGR_PREV_SITE_SSID_PARAM                       = SITE_MGR_MODULE_PARAM | 0x0C,
    SITE_MGR_PREV_SITE_CHANNEL_PARAM                    = SITE_MGR_MODULE_PARAM | 0x0D,
    SITE_MGR_DESIRED_RSSI_GAP_THR_PARAM                 = SITE_MGR_MODULE_PARAM | 0x0E,
    SITE_MGR_PRIORITY_PARAM                             = SITE_MGR_MODULE_PARAM | 0x0F,
 
    /* Regulatory Domain section */
    REGULATORY_DOMAIN_DISCONNECT_PARAM                  = REGULATORY_DOMAIN_MODULE_PARAM| 0x01,
    REGULATORY_DOMAIN_TX_POWER_AFTER_SELECTION_PARAM    = REGULATORY_DOMAIN_MODULE_PARAM| 0x02,
    REGULATORY_DOMAIN_COUNTRY_PARAM                     = REGULATORY_DOMAIN_MODULE_PARAM| 0x03,
    REGULATORY_DOMAIN_POWER_CAPABILITY_PARAM            = REGULATORY_DOMAIN_MODULE_PARAM| 0x04,
    REGULATORY_DOMAIN_SUPPORTED_CHANNEL_PARAM           = REGULATORY_DOMAIN_MODULE_PARAM| 0x05,
    REGULATORY_DOMAIN_SET_POWER_CONSTRAINT_PARAM        = REGULATORY_DOMAIN_MODULE_PARAM| 0x06,
    REGULATORY_DOMAIN_IS_CHANNEL_SUPPORTED              = REGULATORY_DOMAIN_MODULE_PARAM| 0x07,
    REGULATORY_DOMAIN_EXTERN_TX_POWER_PREFERRED         = REGULATORY_DOMAIN_MODULE_PARAM| 0x08,
    REGULATORY_DOMAIN_SET_CHANNEL_VALIDITY              = REGULATORY_DOMAIN_MODULE_PARAM| 0x09,
    REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES             = REGULATORY_DOMAIN_MODULE_PARAM| 0x0a,
    REGULATORY_DOMAIN_ALL_SUPPORTED_CHANNELS            = REGULATORY_DOMAIN_MODULE_PARAM| 0x0b,
    REGULATORY_DOMAIN_CHECK_COUNTRY_PARAM               = REGULATORY_DOMAIN_MODULE_PARAM| 0x0c,
	REGULATORY_DOMAIN_IS_COUNTRY_FOUND					= REGULATORY_DOMAIN_MODULE_PARAM| 0x0d,
    
    /* measurement section */
    MEASUREMENT_TRAFFIC_THRESHOLD_PARAM                 = MEASUREMENT_MODULE_PARAM| 0x01,
    MEASUREMENT_GET_STATUS_PARAM                        = MEASUREMENT_MODULE_PARAM| 0x02,

#ifdef EXC_MODULE_INCLUDED

    EXC_CCKM_EXISTS                                     = EXC_MANAGER_MODULE_PARAM | 0x01,
    EXC_NEIGHBOR_APS                                    = EXC_MANAGER_MODULE_PARAM | 0x02,
#endif

    
}   internalParam_e;


#endif /* __PARAM_IN_H__ */

