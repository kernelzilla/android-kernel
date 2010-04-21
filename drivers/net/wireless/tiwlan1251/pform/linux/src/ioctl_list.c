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


#include "arch_ti.h"

#include <asm/uaccess.h>        /* copy_to_user() */
#include <linux/netdevice.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/vmalloc.h>

#include "esta_drv.h"
#include "tiwlan_profile.h"
#include "ioctl_init.h"
#include "ioctl_utils.h"
#include "tiioctl.h"
#include "ipc_k.h"


void print_priv_ioctl_params(struct net_device *dev, tiioctl_req_t *req, char *extra)
{
        print_deb(" priv_ioctl_params(*dev:%p,*req:%p, *extra:%p)\n", dev, req, extra);
        print_deb("   wrqu.point: user_data=%p, length=%ld, cmd=%ld\n", (void *) req->user_data_pointer,
                  req->length, req->cmd );
        print_deb("   wrqu dump: ");
        print_memory_dump((char *) req, sizeof(*req) );
        print_deb("\n");

        if( extra )
        {
                print_deb("   extra (%p) :", extra );
                print_memory_dump(extra, req->length );
                print_deb("\n");
        }
}

/*sends complete to the user after to signal the completion of the asynchronous */
/*operation (need to set *pIoCompleteFlag = FALSE, at osCmd.c).*/

void os_IoctlComplete(PTIWLN_ADAPTER_T pAdapter, TI_STATUS ReturnStatus )
{
	*pAdapter->pCompleteReply = (int)ReturnStatus;
    complete(pAdapter->IoctlComp);
}


NTSTATUS DispatchCommand(PTIWLN_ADAPTER_T pAdapter,ULONG ioControlCode,PULONG outBufLen,
                    ULONG inBufLen,PVOID ioBuffer,PUINT8 pIoCompleteFlag);

int ti1610_ioctl_priv_proc_tl(tiwlan_req_t *req_data)
{
    struct net_device *dev = req_data->drv->netdev;
    tiioctl_req_t *req = (tiioctl_req_t *) req_data->u.req.p1;
    static unsigned int drv_started = 0;
	static UINT8 IoCompleteFlag ;

    ULONG *data = (ULONG *) req_data->u.req.p2;

    int res = -EINVAL;

    print_deb("priv_ioctl_proc(): cmd=%ld, data=%p (user_data=%lx), lenght=%ld\n",
                req->cmd, data, req->user_data_pointer, req->length);
    if( !drv_started && (req->cmd != TIWLN_DRIVER_STATUS_SET)) { /* Dm: Fix */
            return res;
    }
					
    switch( req->cmd ) {
        case TIWLN_DRIVER_STATUS_SET:
            if(*data)
                res = tiwlan_start_drv( (tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev) );
            else
                res = tiwlan_stop_drv( (tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev) );

            if( res == OK )
                    drv_started = !drv_started;
            break;

        case TIWLN_SEND_EAPOL_PACKET:
            res = os_sendPacket(dev, data, req->length);
            break;
#ifdef TI_DBG
        case TIWLN_DRIVER_DEBUG_PRINT:
                res = util_hal_debug_print(dev, data);
                break;
#endif /* TI_DBG */
        default:
		{
            res = DispatchCommand(&req_data->drv->adapter, req->cmd, &req->length, req->length, data,&IoCompleteFlag );
			/* If we do not have to send complete to user back then set the Falg to FALSE 
			The Complete will be sent from another contect of command completion from FW */
			if(IoCompleteFlag == FALSE)
			{
			   req_data->u.req.reply_expected = FALSE;
			   /****** TO DO - This solution will have a problem in case of two async ioctrls (in case of two utility adapters). ******/
			   /* Store the semaphore for later competion */
			   (req_data->drv->adapter).IoctlComp = &(req_data->u.req.comp);
			   /* Store the pointer of the result status for later competion */
			   (req_data->drv->adapter).pCompleteReply = &(req_data->u.reply);
			}

		}
    }
    return res;
}

int ti1610_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    tiioctl_req_t *req = (tiioctl_req_t *) &rq->ifr_ifru;
    char *extra, *kbuf = NULL;
    int res, aval_data_size = ((char *) req + sizeof(*req)) - (char *)&req->user_data_pointer;      /* = ~4 bytes */
    /*int is_get_cmd = (req->cmd_type & IOCTL_GET);*/

    print_deb("ti1610_do_ioctl(cmd=%lu(%s%s)) - user_data_pointer=0x%lx, len = %lu, aval_data_size=%d\n",
                req->cmd,
                (req->cmd_type & IOCTL_GET) ? "GET" : "", (req->cmd_type & IOCTL_SET) ? "SET" : "",
                req->user_data_pointer, req->length, aval_data_size );

	/* driver is already initialized */
	if ((req->cmd == TIWLN_SET_INIT_INFO) && (((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev))->adapter.CoreHalCtx))
	{
		return 0;
	}

	if( req->length > aval_data_size )
    {
        if( req->user_data_pointer == 0 )
            return -EFAULT;

        print_deb("ti1610_do_ioctl() - alloc %ld bytes\n", req->length );
		kbuf = extra = os_memoryAlloc(NULL,req->length);
#ifdef TI_MEM_ALLOC_TRACE        
        os_printf("MTT:%s:%d ::kmalloc(%lu, %x) : %lu\n", __FUNCTION__, __LINE__, req->length, GFP_KERNEL, req->length);
#endif/*I_MEM_ALLOC_TRACE*/

        if( !extra )
            return -ENOBUFS;
        if( req->cmd_type & IOCTL_SET )
        {
            if( copy_from_user(extra, (void *) req->user_data_pointer, req->length) )
                return -EFAULT;
        }
        else {
            os_memoryZero( NULL, extra, req->length );
        }
    } else
            extra = (char *) &req->user_data_pointer;

    /* Driver initialization must be performed in process context.
       The rest is handled in the context of dedicated tasklet
    */
    if (req->cmd == TIWLN_SET_INIT_INFO)
    {
       tiwlan_dev_init_t *init_info = (tiwlan_dev_init_t *)extra;
       print_deb("TIWLN_SET_INIT_INFO: el=%d il=%d, fl=%d\n",
              init_info?init_info->eeprom_image_length:0,
              init_info?init_info->init_file_length:0,
              init_info?init_info->firmware_image_length:0 );
       res = tiwlan_init_drv((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev), init_info);
    }

#ifdef DRIVER_PROFILING
    else if (req->cmd == TIWLAN_PROFILING_REPORT) 
    {
       res = tiwlan_profile_report((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev));
    }
    else if (req->cmd == TIWLAN_PROFILING_CPU_ESTIMATOR_CMD) {
       /* get the command cpu estimator command parameter */
       unsigned int command_param = *((unsigned int *)extra);
       /* extract the command type which is the MSB byte of the command param*/
       unsigned int command_type = 0xFF & (command_param >> 24);
       /* extract the data of the command which are the 3 LSB bytes of the command param */
       unsigned int command_data = 0xFFFFFF & command_param;
       /* execute the command according to its type */
       switch (command_type) 
       {
       case TIWLAN_PROFILING_CPU_ESTIMATOR_CMD_START:
           res = tiwlan_profile_cpu_usage_estimator_start((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev),
                                                          /* the data in this case is the estimator
                                                          resolution in milliseconds */
                                                          command_data * 1000);
           break;
       case TIWLAN_PROFILING_CPU_ESTIMATOR_CMD_STOP:
           res = tiwlan_profile_cpu_usage_estimator_stop((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev));
           break;
       case TIWLAN_PROFILING_CPU_ESTIMATOR_CMD_RESET:
           res =tiwlan_profile_cpu_usage_estimator_reset((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev));
           break;
       default:
           res = 0;
           printk("\n\n%s: cpu usage estimator unknow command: param = %x\n\n\n",
                     __FUNCTION__, command_param);
       }
    }
#endif

    else
    {
       res = tiwlan_send_wait_reply((tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev), ti1610_ioctl_priv_proc_tl,
                                    (unsigned long)req, (unsigned long)extra, 0, 0);
    }

    if( !res )
    {
            if( (req->cmd_type & IOCTL_GET) && kbuf /*req->length > aval_data_size*/ )
            {
                print_deb("ti1610_do_ioctl(): ...copy from %p to %p %ld bytes\n\n", extra, (void *) req->user_data_pointer, req->length );
                print_memory_dump(extra, min(32,(int) req->length) );
                if( copy_to_user( (void *) req->user_data_pointer, extra, req->length ) )
                    return -EFAULT;
            }
    }
    print_deb("ti1610_do_ioctl() = %d (req = %p, user_data_pointer=0x%lx, extra=%p)\n\n", res, req, req->user_data_pointer, extra );

    if( kbuf ){
		os_memoryFree(NULL,kbuf,sizeof(kbuf)); 
#ifdef TI_MEM_ALLOC_TRACE        
        os_printf("MTT:%s:%d ::kfree(0x%p) : %d\n", __FUNCTION__, __LINE__, kbuf, -req->length);
#endif/*I_MEM_ALLOC_TRACE*/
    }
    return res;
}
