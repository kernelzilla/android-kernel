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
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "osTIType.h"
/*#include "cu_ipc.h"*/

const char* socket_name = "/var/run/echo_server";
struct sockaddr_un serv_addr, clnt_addr;
int clnt_sfd;
/***************************************************************************/
void config_ipc_init(char* device_name)
{
	int serv_addr_len, clnt_addr_len;

	/* Store the client name in the socket address. */
	/*bzero(serv_addr,sizeof(serv_addr));*/
	serv_addr.sun_family = AF_LOCAL;
	strcpy (serv_addr.sun_path, socket_name);

	serv_addr_len = sizeof(serv_addr.sun_family)+strlen(serv_addr.sun_path);

    unlink("/var/run/echo_client");

	/* Create the socket. */
	if((clnt_sfd = socket(AF_LOCAL, SOCK_DGRAM, 0))<0)
	{
		printf("--config_ipc_init(): error creating socket\n");
		exit(1);

	}

	/* Store the client name in the socket address. */
	/*bzero(clnt_addr,sizeof(clnt_addr));*/
	clnt_addr.sun_family = AF_LOCAL;
	strcpy (clnt_addr.sun_path, "/var/run/echo_client");
	clnt_addr_len = sizeof(clnt_addr.sun_family)+strlen(clnt_addr.sun_path);

	if ( (bind (clnt_sfd, (struct sockaddr *)&clnt_addr, clnt_addr_len )) < 0)
	{
		printf("---config_ipc_init(): Error binding \n");
        unlink(clnt_addr.sun_path);
    }
    /*InitTIS_Manager( device_name );*/
    return;

}
/***************************************************************************/
int ipc_send_tosuppl(void *pData, tiUINT32 size)
{
    int rc;
    /*printf("---ipc_send_tosuppl(): %x", size);*/
    int serv_addr_len = sizeof(serv_addr.sun_family)+strlen(serv_addr.sun_path);
	rc = sendto(clnt_sfd, pData, size, 0,
			(struct sockaddr *)&serv_addr, serv_addr_len );
	if( rc < 0 )
	{
		printf("---ipc_send_tosuppl():  Error sending message - %x(%s)\n",errno, strerror(errno));
        unlink(clnt_addr.sun_path);

	}

    /*printf("--ipc_send_tosuppl():  Message has been sent - %x\n",rc);    */
    return rc;
}
/***************************************************************************/
void config_ipc_terminate(void)
{
    unlink("/var/run/echo_client");
    /*TerminateTIS_Manager() //TIWLN_SUPPL_TERMINATE*/
}


