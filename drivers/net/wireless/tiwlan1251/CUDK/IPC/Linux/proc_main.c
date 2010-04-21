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

/**************************************************************************/
/*                                                                        */
/* MODULE:  proc_main.c                                                 */
/* PURPOSE: Supplicant Initialization			                  */
/*	    		                                                  */
/**************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <fcntl.h>      /* for O_WRONLY*/

 #include <sys/time.h>
 #include <sys/resource.h>
 #include <unistd.h>

typedef unsigned int    UINT32;     /*nick*/

#include "osTIType.h"

#include "TI_SupplicantStub.h"
#include "TI_IPC_Api.h"
#include "ipc_event.h"



extern tiBOOL      g_bTerminate ;

int main(int argc, char ** argv)
{
#ifdef TI_DBG
    struct rlimit rLimit;

    rLimit.rlim_cur = 1024*1024;
    rLimit.rlim_max = 1024*1024;

    setrlimit(RLIMIT_CORE,&rLimit);
#endif

    FILE *f = fopen("/dev/console", "w" );
    int hfile_output = -1;
    if( !f )
        fprintf(stderr, "/dev/console open failed: %d(%s)\n", errno, strerror(errno) );
    else
    {
        if( dup2( f->_fileno, 1 ) == -1 )
        {
            fprintf(f, "dup2(hfile, 1) failed: %d(%s)\n", errno, strerror(errno) );
        }
        if( dup2( f->_fileno, 2 ) == -1 )
        {
            fprintf(f, "dup2(hfile, 2) failed: %d(%s)\n", errno, strerror(errno) );
        }
    }

    if( argc == 2 )
    {
        hfile_output = open((char *) argv[1], O_WRONLY );
        if( hfile_output == -1 )
        {
            perror((char *) argv[1]);
            goto exit;
        }
        if( dup2( 1, hfile_output ) == -1 )
        {
            fprintf(stderr, "dup2(1, hfile_output) failed: %d(%s)\n", errno, strerror(errno) );
        }
        if( dup2( 2, hfile_output ) == -1 )
        {
            fprintf(stderr, "dup2(2, hfile_output) failed: %d(%s)\n", errno, strerror(errno) );
        }
    }
    /* Open socket channel for CLI configuration */
    if(cnfg_open() != 0)
	{
        printf("\nCan't initialize configure socket\n");
        return -1;
    }

    if(IPC_Init())
    {
        printf("---main(): ERROR IPC init\n");

    }

    IPC_CONFIG_PARAMS *pRegistry_config = (IPC_CONFIG_PARAMS*)malloc(sizeof(IPC_CONFIG_PARAMS));

    pRegistry_config->F_ConfigNotification = SendDataStub;

    if(IPC_RegisterConfig((void*)pRegistry_config,
                                                sizeof(IPC_CONFIG_PARAMS)))
    {
        printf("---main(): ERROR registration CONFIG Messages\n");
    }
	free(pRegistry_config);


    for(;g_bTerminate != TRUE;)
    {
        usleep(1000000);     /* sleep 1 sec*/
    }
	
exit:
    if( f > 0 )
        fclose( f );
    if( hfile_output > 0 )
        close( hfile_output );
    return 0;
}


