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


#include "ioctl_init.h"
#include "esta_drv.h"

#include "ioctl_utils.h"

#include "report.h"
#include "osClsfr.h"

#define CONFIG_MGR(dev) ((configMgr_t *) \
    ((dev && dev->priv) ? ((tiwlan_net_dev_t *) dev->priv)->adapter.CoreHalCtx : NULL ) )


typedef struct
{
    int err_code;
    char *desc;
}
configMgr_error_desc_t;


static configMgr_error_desc_t configMgr_error_desc[] =
{
    { OK,		"OK"	},
    { NOK,		"NOK"	},
    { PARAM_NOT_SUPPORTED,	"PARAM_NOT_SUPPORTED"	},
    { PARAM_VALUE_NOT_VALID,	"PARAM_VALUE_NOT_VALID"	},
    { CONFIGURATION_NOT_VALID,	"CONFIGURATION_NOT_VALID"	},
    { NO_SITE_SELECTED_YET,		"NO_SITE_SELECTED_YET"	},
    { RE_SCAN_NEEDED,		"RE_SCAN_NEEDED"	},
    { EXTERNAL_SET_PARAM_DENIED,	"EXTERNAL_SET_PARAM_DENIED"	},
    { EXTERNAL_GET_PARAM_DENIED,	"EXTERNAL_GET_PARAM_DENIED"	},
    { PARAM_MODULE_NUMBER_INVALID,	"PARAM_MODULE_NUMBER_INVALID"	},
    { STATION_IS_NOT_RUNNING,	"STATION_IS_NOT_RUNNING"	},
    { CARD_IS_NOT_INSTALLED,	"CARD_IS_NOT_INSTALLED"	},
    /* Data path section */
    /* 	{ RX_BSS_TYPE_ERROR			,*/
    /* 	{ RX_BSS_ID_ERROR				,*/
    /* 	{ TX_QUEUE_SELECTED_OK		,*/
    /* 	{ NO_TX_QUEUE_SELECTED		,*/
    /* 	{ TX_STATUS_PENDING			,*/
    /* 	{ TX_STATUS_NO_RESOURCES		,*/
    /* 	{ TX_STATUS_FAILURE			,*/
    /* 	{ TX_STATUS_OK				,*/

    /* 4x section */
    /* 	{ MAKE_CONCATENATION			,*/
    /* 	{ SEND_ONE_MSDU				,*/
    /* 	{ DO_NOT_SEND_MSDU			,*/
    /* 	{ FOUR_X_DISABLE				,*/
    /**/
    /* 	 (Scanning section) */
    /* 	{ NO_COUNTRY					,*/
    /**/
    /* 	 (Setting power after select) */
    /* 	{ TX_POWER_SHOULD_NOT_BE_SET	,*/
    /* 	(changing service channel)       */
    /* 	{ CHANNEL_CHANGED				,*/
    /* 	{ SUPPORT_IMMEDIATE_MEASUREMENT_ONLY,*/
    /* 	{ MEASUREMENT_TYPE_NOT_SUPPORT,*/
    /* 	{ MEASUREMENT_CAN_NOT_EXECUTED_IN_PARALLEL,*/
    /* 	{ MEASUREMENT_REQUEST_IGNORED,*/
    /* 	{ CANNOT_SET_MEASUREMENT_PARAM_WHEN_ACTIVATED,*/
    /* 	{ REGULATORY_DOMAIN_SET_TX_POWER_PARAM,*/
    /* 	{ CANNOT_SET_CHANNEL_THAT_IS_NOT_SUPPORTED,*/
};


int print_err_desc(int err)
{
    int i;
    for( i=0; i<SIZE_ARR(configMgr_error_desc);i++ )
    {
        if(configMgr_error_desc[i].err_code == err )
        {
            print_err("---err(%d) configMgr() = %s\n", err, configMgr_error_desc[i].desc );
            return -err;
        }
    }
    print_err("---err(%d) configMgr failed\n", err);
    return -err;
}

void print_memory_dump(char *addr, int size )
{
#ifdef DEBUG_MESSAGES
        int i;
        char buf[4096];

        if( size * 4 > sizeof(buf) ) {
                print_err("print_memory_dump(): buffer too small\n");
                return;
        }

        buf[0] = 0;
        for(i=0; i<size; i++ ) {
                if( !(i % 16) )
                        sprintf(&buf[strlen(buf)], "%s%p: ", (i) ? "\n" : "", addr+i );
                sprintf(&buf[strlen(buf)], "%02x ", (unsigned char)addr[i] );
        }
        print_info("%s\n", buf);
#endif /* DEBUG_MESSAGES */
}

#ifdef TI_DBG
#ifndef TIWLAN_MSM7000
TI_STATUS debugFunction(TI_HANDLE hConfigMgr,
			UINT32 functionNumber,
			void *pParam);
#endif			
#endif
int util_hal_debug_print(struct net_device *dev, ULONG *data)
{
    hal_print_param_t *p = (hal_print_param_t *) data;
    UINT32 opt_data = 0;
    
    if( p->optional_param )
    {
        if( copy_from_user(&opt_data, p->optional_param, sizeof(opt_data) ) )
            return -EFAULT;
    }

    print_deb("HAL_DEBUG_PRINT: func_id=%d, param=%x\n", p->func_id, opt_data );
#ifdef TI_DBG    
    debugFunction(CONFIG_MGR(dev), p->func_id, &opt_data );
#endif
    return 0;
}

