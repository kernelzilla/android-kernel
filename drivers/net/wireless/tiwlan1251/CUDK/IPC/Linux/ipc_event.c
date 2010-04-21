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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <linux/stddef.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>

#include <pthread.h>

#include "ipc_event.h"
#include "cli_cu_common.h"
#include "tiioctl.h"
#include "TI_IPC_Api.h"

static int dev_socket = -1;
static int serv_sfd = -1;

config_registry_t cnfg_registry_table;

UINT32 prId;

static tiINT32  ipc_start = 0;
/******************************************************************
    _ipc_EventHandler
    Driver events handler
*/
tiVOID _ipc_EventHandler(UINT8* pEvMessage,  tiUINT32 nEvMessageSize )
{
/*    print_memory_dump((char*)pEvMessage, nEvMessageSize );*/
	if (((IPC_EVENT_PARAMS*) pEvMessage)->pfEventCallback == NULL)
	{
        printf("\n---_ipc_EventHandler: ERROR Bad Callback pointer\n");
        return;
	}

    if(nEvMessageSize < sizeof(IPC_EVENT_PARAMS))
    {
        print_err("\n---_ipc_EventHandler: ERROR event size - %x\n ",nEvMessageSize);
        return;
    }
    print_deb("--_ipc_EventHandler: \n");
    ((IPC_EVENT_PARAMS*) pEvMessage)->pfEventCallback((IPC_EV_DATA * )pEvMessage);    
}
/*********************************************************************
    _ipc_cu_handler
    CLI configuration events handler
*/
tiVOID _ipc_cu_handler(UINT8* pMessage,  tiUINT32 nMessageSize)
{
    print_deb("\n---_ipc_cu_handler() data size - %d\n ",nMessageSize);

    print_memory_dump((char*)pMessage, nMessageSize );
    IPC_CONFIG_PARAMS *pData_tosend = (IPC_CONFIG_PARAMS *)malloc(sizeof(IPC_CONFIG_PARAMS));

    pData_tosend->F_ConfigNotification = cnfg_registry_table.cfg_cb;

    (*pData_tosend->F_ConfigNotification)(pMessage, nMessageSize );

    free(pData_tosend);
}
/*********************************************************************/

void check_unbound(void)
{
/*
    OS_802_11_MAC_ADDRESS curr_mac = { 0 };
    UINT32  size;
    if(IPC_DeviceIoControl((TI_HANDLE)ti_drv_name, TIWLN_802_3_CURRENT_ADDRESS, NULL, 0, &curr_mac,
                                    sizeof(OS_802_11_MAC_ADDRESS),  (tiUINT32*)nSize ))
    {
        send unbound

    }
 */
}
/**************************************************************/
void *_ipc_EventThread(void *args)
{
	/* set timeout for select*/
	int fd;
    char* buf = (char*)malloc(4096); /* the maximum acceptable size*/
    fd_set fds_read;
	struct nlmsghdr * nlHdr;
	UINT8 * pData;
	tiUINT32 nDataLen;

	if (dev_socket > serv_sfd)
		fd = dev_socket+1;
	else
        fd = serv_sfd+1;

	print_deb("---_ipc_EventThread() - thread is running fd = %d\n", fd);

	while(1)
	{
		/* create fd_set*/
		FD_ZERO(&fds_read);

        if (dev_socket != -1) 
            FD_SET(dev_socket, &fds_read);
		
        if (serv_sfd != -1) 
            FD_SET(serv_sfd, &fds_read);

        /* wait for packet on two sockets*/
        int n = select(fd, &fds_read, NULL, NULL,NULL );

        /* if data is available, receive it*/
        if( n > 0 )
        {
			/* this is event from the driver*/
			if( FD_ISSET(dev_socket, &fds_read) )
			{
			    /* get 'n' bytes from driver socket*/
                n = recvfrom(dev_socket, buf, 4096, 0, NULL,NULL/*(struct sockaddr*)&sanl, &sanllen*/);
                if (n <= 0)
                {
                    perror(__FUNCTION__);
                    continue;
                }
				nlHdr = (struct nlmsghdr *)buf;

				/* Check if header is valid */
				if((NLMSG_OK(nlHdr, n) == 0) || (nlHdr->nlmsg_type == NLMSG_ERROR)){
					perror("Error in recieved NetLink packet");
					continue;
				}
				pData = (UINT8 *)NLMSG_DATA(nlHdr);
				nDataLen = NLMSG_PAYLOAD(nlHdr,n);
				_ipc_EventHandler (pData, nDataLen );

            }

			else if( FD_ISSET(serv_sfd, &fds_read) )
			{
                
                print_deb(" ---_ipc_EventThread() cu receive socket: %x \n",
                                                                    serv_sfd);

                /*printf(" ---_ipc_EventThread() cu receive enable cnfg_registry_table.enable\n");*/
				/* read, get # bytes read*/
				    
				    n = recvfrom(serv_sfd, buf, 4096, 0,NULL,NULL);
				    if(n < 0)
				    {
					    print_err("--- _ipc_EventThread() -ERROR receiving config msg \n");
                        continue;
				    }
				    else
                        _ipc_cu_handler((UINT8*) buf, n);
			}

            else
                print_err(" ---_ipc_EventThread() cu not a socket: %x \n",serv_sfd);
        }
    }

    print_err("IPC exits \n");

    return 0;
}


/******************************************************************************/
tiINT32 rtnl_open(void)
{

    int fd;
    struct sockaddr_nl      local;
    prId = getpid();

    /*printf("PROCCESS: %d\n", prId);*/

    fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_USERSOCK);
    if (fd < 0) {
         perror(__FUNCTION__);
         return -1;
     }

     memset(&local, 0, sizeof(local));
     local.nl_family = AF_NETLINK;
     /*    local.nl_groups = RTMGRP_LINK;*/
     local.nl_pid = prId;

     if (bind(fd, (struct sockaddr*)&local, sizeof(local)) < 0) {
         perror(__FUNCTION__);
         return -1;
     }
     /*printf("User: socket - %d\n",fd);*/
     return fd;


}
/*****************************************************************************/
tiINT32 cnfg_open(void)
{
	/*int rc;*/
	struct sockaddr_un serv_addr;/* clnt_addr;*/

	int serv_addr_len;/* clnt_addr_len, max_clnt_len, n;*/

    unlink("/var/run/echo_server");
	/* Create socket */
	if(( serv_sfd = socket(AF_LOCAL, SOCK_DGRAM, 0)) < 0)
	{
		print_err("--Server: error creating socket\n");
		return -1;
	}
    print_deb("---cnfg_open(): socket - %d\n", serv_sfd);
	/*bzero(serv_addr,sizeof(serv_addr));*/
	/* Store the client s name in the socket address. */
	serv_addr.sun_family = AF_LOCAL;
	strcpy (serv_addr.sun_path, "/var/run/echo_server");

	serv_addr_len = sizeof(serv_addr.sun_family)+strlen(serv_addr.sun_path);

	if(bind(serv_sfd, (struct sockaddr*)&serv_addr, serv_addr_len) <0 )
	{
		print_err("--Server: error binding to server socket - err %d(%s)\n", errno, strerror(errno));
		return -1;
	}
	return 0;
}
/*****************************************************************************/
tiINT32 ipc_interfaces_init(tiVOID)
{
	int                 rc;
    pthread_t           ev_thread_id;


	/* Open netlink channel */
    if((dev_socket = rtnl_open()) < 0)
    {
      printf("\nCan't initialize rtnetlink socket\n");
      return -1;
    }
    print_deb("---ipc_interfaces_init - rtnetlink socket  is created \n");
    rc = pthread_create(&ev_thread_id, NULL, _ipc_EventThread, (void*)NULL);

    print_deb("---ipc_CreateInterface() - thread is created \n");
	if (rc)
		printf("---ipc_CreateInterface() - ERROR code from pthread_create() is %d\n", rc);
	return 0;
}
/******************************************************************************/
tiINT32 IPC_RegisterEvent(TI_HANDLE hDevice, IPC_EVENT_PARAMS* 	pEvParams)
{

    /*set ioctl event enable*/
    IPC_EVENT_PARAMS* pReg = (IPC_EVENT_PARAMS*)pEvParams;
    IPC_EVENT_PARAMS ouput;
    tiINT32 res;
    tiUINT32 bytesReturned;

    if(pReg->uEventType == IPC_EVENT_UNBOUND)/*insert UNBOUND*/
    {
		return 0;
    }


    pEvParams->uDeliveryType = DELIVERY_PUSH;
    pEvParams->uProcessID = prId;
    print_deb("---IPC_RegisterEvent() event - %#x\n",  pReg->uEventType);
    res = IPC_DeviceIoControl(hDevice, TIWLN_802_11_ENABLE_EVENT, (tiVOID*)pEvParams, sizeof(IPC_EVENT_PARAMS), /*NULL,0*/&ouput, sizeof(IPC_EVENT_PARAMS), &bytesReturned );
    pReg->uEventID = ouput.uEventID;
    return res;

}
/******************************************************************************/
tiINT32 IPC_UnRegisterEvent(TI_HANDLE hDevice, IPC_EVENT_PARAMS* pEvParams)
{
    UINT32 id = (UINT32)pEvParams->uEventID;
    tiUINT32 bytesReturned;

    /*set ioctl event disable*/
    return IPC_DeviceIoControl(hDevice, TIWLN_802_11_DISABLE_EVENT, (tiVOID*)&id, sizeof(UINT32), NULL, 0, &bytesReturned );
}
/******************************************************************************/
tiINT32 IPC_RegisterConfig(tiVOID* pEvParams, tiUINT32 EvParamsSize)
{
    /*set ioctl event enable*/

    IPC_CONFIG_PARAMS	*pData      = (IPC_CONFIG_PARAMS*)pEvParams;

    cnfg_registry_table.len = EvParamsSize;
    cnfg_registry_table.cfg_cb = pData->F_ConfigNotification;
    cnfg_registry_table.enable = 1;
    print_deb("---IPC_RegisterConfig() l - %x \n", cnfg_registry_table.len);
    return 0;

}

/*****************************************************************************/
TI_HANDLE IPC_Init(void)
{
    int rc = 0;
    if(ipc_start)
        return (TI_HANDLE)rc;
    else
    {
        rc = ipc_interfaces_init();
        ipc_start++;
    }
    return (TI_HANDLE)rc;

}
/*****************************************************************************/
tiINT32 IPC_DeInit (void)
{
   /* close(dev_socket );*/
   /* close(serv_sfd );*/
    return 0;
}
