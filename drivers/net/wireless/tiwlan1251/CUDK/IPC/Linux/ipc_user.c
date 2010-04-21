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
#include <string.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <stdlib.h>

#include <unistd.h>

#include "tiioctl.h"

#include "osDot11.h"
#include "linux_ioctl_common.h"
#include "cli_cu_common.h"

#include "TI_IPC_Api.h"


static int     g_socket = 0;                /* socket descriptor */

static int iw_sockets_open(void)
{
/*    int netlink_sock = -1;  */      /* Kernel user interface device */
    int ipx_sock = -1;              /* IPX socket                   */
    int ax25_sock = -1;             /* AX.25 socket                 */
    int inet_sock = -1;             /* INET socket                  */
    int ddp_sock = -1;              /* Appletalk DDP socket         */

        /*
         * Now pick any (exisiting) useful socket family for generic queries
     * Note : don't open all the socket, only returns when one matches,
     * all ocols might not be valid.
     * Workaround by Jim Kaba <jkaba@sarnoff.com>
     * Note : in 99% of the case, we will just open the inet_sock.
     * The remaining 1% case are not fully correct...
         */
    /*netlink_sock=socket(PF_NETLINK, SOCK_DGRAM, 0);
    if(netlink_sock!=-1)
        return netlink_sock;*/
    inet_sock=socket(AF_INET, SOCK_DGRAM, 0);
    if(inet_sock!=-1)
        return inet_sock;
    ipx_sock=socket(AF_IPX, SOCK_DGRAM, 0);
    if(ipx_sock!=-1)
        return ipx_sock;
    ax25_sock=socket(AF_AX25, SOCK_DGRAM, 0);
    if(ax25_sock!=-1)
        return ax25_sock;
    ddp_sock=socket(AF_APPLETALK, SOCK_DGRAM, 0);
    /*
    * If this is -1 we have no known network layers and its time to jump.
    */
    return ddp_sock;
}


TI_HANDLE IPC_DeviceOpen (tiVOID* pAdapterName)
{
   g_socket = iw_sockets_open();
   if( g_socket == -1 )
   {
       perror("socket");
       return 0;
   }    
    /* create interfaces for events receiving */
    /*
    if (_ipc_CreateInterface(pAdapterName))
    {
        print_err("**IPC error**\n");
        return NULL;
    }
    */
    return (pAdapterName && *((char *) pAdapterName)) ? (TI_HANDLE) pAdapterName : NULL;
}

tiINT32 IPC_DeviceClose(TI_HANDLE hDevice)
{
    if( g_socket )
           close( g_socket );
    return 0;
}

tiINT32 IPC_DeviceIoControl(TI_HANDLE hDevice, tiUINT32 ioctl_cmd, tiVOID* bufIn, tiUINT32 sizeIn, 
                                                tiVOID* bufOut, tiUINT32 sizeOut, tiUINT32* sizeRet  )

{
    int res, max_size, cmd;
    struct ifreq req;
    tiioctl_req_t *ti_req = (tiioctl_req_t *) &req.ifr_ifru;
    tiVOID* buf, *setget_buf = NULL;
    tiUINT32 size;
    int cmd_type = ((bufOut) ? IOCTL_GET : 0) | ((bufIn) ? IOCTL_SET : 0);
    if( !g_socket )
    {
        print_err("**Socket error**\n");
        return -EINVAL;
    }
    
    if( (cmd_type & IOCTL_SETGET) == IOCTL_SETGET )
    {
        size = max( max(sizeIn, sizeOut ), 8 );        /* always pass as pointer to buffer*/
        if( size >= 0xFFFF )
        {
            print_err("Max. size for SET_GET ioctl - %u bytes (sizeIn=%u, sizeOut=%u)\n",
                    0xFFFF, sizeIn, sizeOut );
            return -ENOMEM;
        }
        buf = setget_buf = malloc(size);
        if( !setget_buf ){
			print_err("IPC_DeviceIoControl: setget_buf is NULL\n");
            return -ENOMEM;
		}
        memmove(setget_buf, bufIn, sizeIn );
        
        cmd = SIOCDEVPRIVATE;
    }
    else
    {
        if( cmd_type & IOCTL_GET )
        {
            buf = bufOut;
            size = sizeOut;
            cmd = SIOCDEVPRIVATE+1;        /* for GET private ioctls */
        }
        else
        {
            buf = bufIn;
            size= sizeIn;
            cmd = SIOCDEVPRIVATE;        /* for SET private ioctls */
        }
    }
#if DEBUG    
    memset(&req, 0xfe, sizeof(req) );
#endif    
    print_deb("===IPC_DeviceIoControl('%s', cmd=%u (%s%s), data=%p (%#x), size=%u, sizeRet = %p(%x))\n", 
            (char *) hDevice, ioctl_cmd, 
            (cmd_type & IOCTL_SET) ? "SET" : "", (cmd_type & IOCTL_GET) ? "GET" : "", 
            buf, (buf) ? * (tiINT32 *)buf : 0, size, sizeRet,  sizeRet ? *sizeRet : -1);

    max_size = ((char *) &req + sizeof(req)) - (char *) &ti_req->user_data_pointer;
    
    ti_req->length      = size;
    ti_req->cmd         = ioctl_cmd;
    ti_req->cmd_type    = cmd_type;

    if( size > max_size )
    {
        ti_req->user_data_pointer     = (tiUINT32) buf;
    }
    else
    {
        if( cmd_type & IOCTL_SET )         /* SET ioctl */
        {
            /*print_deb("util_get_ioctl(): offset=%d, data='%s', size=%d\n", offset, (char *)data, size);*/
            memmove( ((tiCHAR *) &ti_req->user_data_pointer), buf, size );
        }
        else 
            ti_req->user_data_pointer = 0;
    }

    strncpy(req.ifr_ifrn.ifrn_name, (char *) hDevice, IFNAMSIZ);

    print_deb("===== send_ioctl: socket=%d, cmd=%d, &req=%p\n", g_socket, cmd, &req );
    print_deb("===== send_ioctl: req={name='%s', sub_cmd=%ld, size=%ld (max=%d), data=%p\n",
                req.ifr_ifrn.ifrn_name, ti_req->cmd, ti_req->length, max_size, (void *) ti_req->user_data_pointer );

/*    print_memory_dump( (char *) &req, sizeof(req));*/
    res = ioctl(g_socket, cmd, &req);
#ifdef GWSI_DRIVER
	/* Yuck - set the result to 0 in case ioctl return error */ 
	if (res == -1) res = 0;
#endif
	if( res )
    {            /* for DEBUG version ONLY*/
        print_deb( "** IPC_DeviceIoControl() return %d\n", res /*(char *) hDevice*/ );
/*        return res;*/
    }
    else
    {
        if( cmd_type & IOCTL_GET )
        {
			size = ti_req->length;

            if( (cmd_type & IOCTL_SETGET) == IOCTL_SETGET )
            {
				memmove(bufOut,setget_buf,size);
                free( setget_buf );                
            }
            else
            {
                if( size <= max_size )
                    memmove( buf, (char *) &ti_req->user_data_pointer, size );
                print_memory_dump((char *) buf, min(32, size) );
                print_deb( "IPC_DeviceIoControl(): Size=%u, max_size=%d, *data=%x\n", size, max_size, * (tiUINT32*) buf );
            }
        }
        
        if( sizeRet )
            *sizeRet = size;    /* ti_req->length */
    
        print_deb("===== send_ioctl: res=%d, sizeRet=%p (%x)\n", res, sizeRet, sizeRet ? *sizeRet : -1);
    }

    return res;
}


/* --------------------------------------------------------- */



/* tiINT32 TI_hwReset (TI_HANDLE  hAdapter)*/
/* {*/
/*     tiUINT32 data = 1;*/
/*     return IPC_DeviceIoControl( hAdapter, TIWLN_HW_RESET_HW, IOCTRL_SET, &data, sizeof(data) );*/
/*      return tiwlan_get_ioctl(hAdapter, -1, TIWLN_HW_RESET_HW, &data, sizeof(data) ); */
/* }*/

/* tiINT32 TI_NumberOfAntennas (TI_HANDLE hAdapter, tiUINT32* puNumberOfAntennas )*/
/* {*/
/*     return IPC_DeviceIoControl( hAdapter, TIWLN_802_11_NUMBER_OF_ANTENNAS, IOCTRL_GET, puNumberOfAntennas, sizeof(tiUINT32) );*/
/* }*/


/* int tiwlan_set_mixed_mode( U32 drv_handler, U32 data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_MIXED_MODE_SET, &data, sizeof(data) );*/
/* }*/
/* */
/* int tiwlan_get_mixed_mode( U32 drv_handler, U32 *p_data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_MIXED_MODE_GET, p_data, sizeof(*p_data) );*/
/* }*/
/* */
/* int tiwlan_set_privacy_mode( U32 drv_handler, U32 data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_PRIVACY_MODE_SET, &data, sizeof(data) );*/
/* }*/
/* */
/* int tiwlan_get_privacy_mode( U32 drv_handler, U32 *p_data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_PRIVACY_MODE_GET, p_data, sizeof(*p_data) );*/
/* }*/
/* */
/* int tiwlan_set_exc_security_type( U32 drv_handler, U32 data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_EXC_SECURITY_TYPE_SET, &data, sizeof(data) );*/
/* }*/
/* */
/* int tiwlan_get_exc_security_type( U32 drv_handler, U32 *p_data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_EXC_SECURITY_TYPE_GET, p_data, sizeof(*p_data) );*/
/* }*/
/* int tiwlan_set_tx_power_val( U32 drv_handler, U32 data )*/
/* {*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_TX_POWER_VALUE_SET, &data, sizeof(data) );*/
/* }*/
/* */
/* int tiwlan_get_tx_power_val( U32 drv_handler, U32 *p_data )*/
/* {*/
/*     print_deb("===GET_TX_POWER_VAL: %d\n", TIWLN_TX_POWER_VALUE_GET );*/
/*     return tiwlan_get_ioctl( g_id_adapter, -1, TIWLN_TX_POWER_VALUE_GET, p_data, sizeof(*p_data) );*/
/* }*/
/* */

/* int tiwlan_get_current_mac( TI_HANDLE drv_handler, OS_802_11_MAC_ADDRESS *data)*/
/* {*/
/*     return IPC_DeviceIoControl(drv_handler, TIWLN_802_3_CURRENT_ADDRESS, NULL, 0, data, sizeof(*data), NULL);*/
/* }*/

/* int tiwlan_get_current_channel(TI_HANDLE drv_handler, tiUINT32 *data)*/
/* {*/
/*     return IPC_DeviceIoControl( drv_handler, TIWLN_802_11_CHANNEL_GET, NULL, 0, data, sizeof(data), NULL );*/
/* }*/

/* int tiwlan_get_desired_ssid(TI_HANDLE drv_handler, char *data)*/
/* {*/
/*     OS_802_11_SSID ssid = { 0 };*/
/*     int res = IPC_DeviceIoControl( drv_handler, TIWLN_802_11_DESIRED_SSID_GET, NULL, 0, &ssid, sizeof(OS_802_11_SSID), NULL );*/
/*     if( !res )*/
/*     {*/
/*         memmove(data, ssid.Ssid, ssid.SsidLength );*/
/*         data[ssid.SsidLength] = 0;*/
/*     }*/
/*     return res;*/
/* }*/


void print_memory_dump(char *addr, int size )
{
    int i;
    char buf[4096];

    if( size * 4 > sizeof(buf) )
    {
        print_err("print_memory_dump(): buffer too small\n");
        return;
    }

    buf[0] = 0;
    for(i=0; i<size; i++ )
    {
        if( !(i % 16) )
            sprintf(&buf[strlen(buf)], "%sTI_CU:%p: ", (i) ? "\n" : "", addr+i );
        sprintf(&buf[strlen(buf)], "%02x ", (unsigned char) addr[i] );
    }
    print_deb("%s\n", buf);
}

