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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/netlink.h>

#include "osTIType.h"
#include "esta_drv.h"
#include "osApi.h"
#include "ioctl_init.h"
#include "cli_cu_common.h"
#include "TI_IPC_Api.h"

UINT32 IPCKernelInit    (TI_HANDLE hAdapter,TI_HANDLE  hIPCEv)
{
    return 0;
}

UINT32 IPCKernelDeInit  (TI_HANDLE hAdapter)
{
    return 0;
}


/*******************************************************/
INT32 IPC_EventSend(TI_HANDLE hAdapter, tiUINT8* pEvData, UINT32 EvDataSize)
{
	 struct sk_buff *skb;
     int res;
     tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *) hAdapter;
     UINT32 realSize = 0;
     UINT32 msgSize;
	 struct nlmsghdr *nlh;
     tiUINT8 *msg;

     /* This event is targetted to the OS process Id 0 is not a valid pId for LINUX*/

     if ((( IPC_EVENT_PARAMS *) pEvData) ->uProcessID == 0) 
     {
         (( IPC_EVENT_PARAMS *) pEvData) ->pfEventCallback(( IPC_EV_DATA *) pEvData);
         return 0;
     }

     /* set the payload size */
	 msgSize = (( IPC_EV_DATA *) pEvData) ->uBufferSize + offsetof(IPC_EV_DATA,uBuffer);
     
	 /* add the netlink header size */
	 realSize = NLMSG_SPACE(msgSize);

	 /* allocate the complete message */
	 skb = dev_alloc_skb(realSize);
	 if (!skb) {
		printk(KERN_ERR "Failed to allocate new skb with size=%u.\n",realSize);
		return -1;
	 }

	 /* set the netlink header params */
	 nlh = NLMSG_PUT(skb, 0, 0, NLMSG_DONE, realSize - sizeof(*nlh));
	 
	 /* get the payload pointer */
	 msg = (char *)NLMSG_DATA(nlh);
	 
	 /* copy the data to the payload */ 
     memcpy(msg,pEvData,msgSize);

	 NETLINK_CB(skb).pid = 0;   /* from kernel */
     NETLINK_CB(skb).dst_group = RTNLGRP_LINK;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
     NETLINK_CB(skb).dst_pid = (( IPC_EVENT_PARAMS *) pEvData) ->uProcessID; /* Dm: */
#endif

	 /* send the message*/
	 res = netlink_unicast(drv->wl_sock, skb, (( IPC_EVENT_PARAMS *) pEvData) ->uProcessID, MSG_DONTWAIT);

     /* Sanity checks. As far as we're concerned this error is unrecovarable.*/
     if (res >= 0)
     {
		return 0;
     }

nlmsg_failure:
    ti_dprintf(TIWLAN_LOG_INFO,"IPC kernel: did not send the netlink message\n");
	return -1;
}
