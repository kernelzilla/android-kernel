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

#include "osrgstry_parser.h"

extern VOID regReadLastDbgState(PTIWLN_ADAPTER_T pAdapter);

static char *init_file     = NULL;
static int init_file_length= 0;
static PNDIS_CONFIGURATION_PARAMETER pNdisParm;

extern  int osInitTable_IniFile(tiwlan_net_dev_t *drv, initTable_t *InitTable, char *file_buf, int file_length)
{
   NDIS_CONFIGURATION_PARAMETER parm;

   init_file         = file_buf;
   init_file_length  = file_length;
   pNdisParm = &parm;

   regFillInitTable( &drv->adapter, InitTable );
#ifdef TI_DBG
   regReadLastDbgState(&drv->adapter);
#endif
   
   return 0;
}

unsigned long TiDebugFlag;

/* void PRINT( char * type, char *format, ... )*/
/* {*/
/*     return ;*/
/* }*/

NDIS_STATUS NdisUnicodeStringToAnsiString( IN OUT PANSI_STRING  DestinationString,
    IN PUNICODE_STRING  SourceString )
{
    if( DestinationString->MaximumLength < SourceString->Length )
        return NDIS_STATUS_BUFFER_TOO_SHORT;

    DestinationString->Length = SourceString->Length;
    os_memoryCopy( NULL, DestinationString->Buffer, SourceString->Buffer, SourceString->Length );
    return NDIS_STATUS_SUCCESS;

}

#ifndef tolower
#define tolower(c)  ( (c) | 0x20)
#endif

    /* Search sub-string in memory buffer */
    /* From '#' to EOL ---- remarks */
char *mem_str(char *buf, char *str, char *end_buf)
{
    int i;

    for( ; buf <= end_buf; buf++ )
    {
        if( *buf == '#' )
        {
            buf = strchr(buf+1, '\n' );
            if( !buf )
                return NULL;

        }
        for( i=0; &buf[i] <= end_buf && buf[i] && str[i] && (tolower(buf[i]) == tolower(str[i])); i++ ) ;

        if ((!str[i]) && (!((tolower(*(buf-1))>='a') && (tolower(*(buf-1))<='z'))))
            return buf;
    }
    return NULL;
}

char * ltrim(char *s )
{
    while( *s == ' ' || *s == '\t' ) s++;
    return s;
}

VOID NdisReadConfiguration( OUT PNDIS_STATUS  status, OUT PNDIS_CONFIGURATION_PARAMETER  *param_value,
    IN NDIS_HANDLE  config_handle, IN PNDIS_STRING  keyword, IN NDIS_PARAMETER_TYPE  param_type )
{
#ifdef USE_INIT_FILE
    char *name = keyword->Buffer;
    char *s, *buf = init_file, *end_buf = init_file + init_file_length;
    static int count = 0;

    *status = NDIS_STATUS_FAILURE;
    *param_value = pNdisParm;

    if( !count )
    {
        print_deb("\n++++++++++++\n%s+++++++++++\n", init_file);
        count++;
    }

    if( !name || !*name || !init_file || !init_file_length )
        return ;

    memset(pNdisParm, 0, sizeof(NDIS_CONFIGURATION_PARAMETER));

    while(buf < end_buf)
    {
        buf = ltrim(buf);
        if( !(s = mem_str(buf, name, end_buf)) )
            break;

        buf = ltrim(s + strlen(name));
        if( *buf == '=' )
            buf++;
        else {
            /*print_err("\n...init_config err: delim not found (=): ** %s **\n", buf );*/
            buf = s + 1; /*strlen(name);*/
            continue;
        }
        buf = ltrim(buf);
        if( param_type == NdisParameterString )
        {
            char *remark = NULL;

            s = strchr(buf, '\n');
            if( !s )
                s = buf+strlen(buf);
            
            remark = memchr(buf, '#', s - buf);        /* skip remarks */
            if( remark )
            {
                do {        /* remove whitespace  */
                    remark--;
                } while( *remark == ' ' || *remark == '\t' );    
                
                pNdisParm->ParameterData.StringData.Length = remark - buf + 1;
            }
            else
                pNdisParm->ParameterData.StringData.Length = s - buf;
                   
            pNdisParm->ParameterData.StringData.Buffer = (PUCHAR)&pNdisParm->StringBuffer[0];
            pNdisParm->ParameterData.StringData.MaximumLength = NDIS_MAX_STRING_LEN;
            if( !pNdisParm->ParameterData.StringData.Length > NDIS_MAX_STRING_LEN )
            {
                *status = NDIS_STATUS_BUFFER_TOO_SHORT;
                return;
            }
            memcpy(pNdisParm->ParameterData.StringData.Buffer, buf, pNdisParm->ParameterData.StringData.Length);
            print_info("NdisReadConfiguration(): %s = (%d)'%s'\n", name, pNdisParm->ParameterData.StringData.Length, pNdisParm->ParameterData.StringData.Buffer);
        }
        else if( param_type == NdisParameterInteger )
        {
	    char *end_p;
            pNdisParm->ParameterData.IntegerData = simple_strtol(buf, &end_p, 0);
            if (end_p && *end_p && *end_p!=' ' && *end_p!='\n'
		&& *end_p!='\r' && *end_p!='\t')
            {
                print_err("\n...init_config: invalid int value for <%s> : %s\n", name, buf );
                return;
            }
            /*print_deb(" NdisReadConfiguration(): buf = %p (%.20s)\n", buf, buf );*/
            print_info("NdisReadConfiguration(): %s = %d\n", name, (INT32) pNdisParm->ParameterData.IntegerData);
        }
        else
        {
            print_err("NdisReadConfiguration(): unknow parameter type %d for %s\n", param_type, name );
            return;
        }
        *status = NDIS_STATUS_SUCCESS;
        return;

    }
/*    print_deb("NdisReadConfiguration(%d): (%c)%s  - not found\n", init_file_length,*/
/*                (param_type == NdisParameterString) ? 'S' : 'D', name );*/
#else
    /* No init file support */
    *status = NDIS_STATUS_FAILURE;

#endif
    return ;
}

VOID NdisWriteConfiguration( OUT PNDIS_STATUS  Status, 
                IN NDIS_HANDLE  ConfigurationHandle,
                IN PNDIS_STRING  Keyword, 
                IN PNDIS_CONFIGURATION_PARAMETER  ParameterValue )
{
    print_err(" NdisWriteConfiguration(): ** not implemented yet ...\n");
}

VOID NdisReadNetworkAddress( OUT PNDIS_STATUS  Status, OUT PVOID  *NetworkAddress, OUT PUINT  NetworkAddressLength,
    IN NDIS_HANDLE  ConfigurationHandle )
{
    print_err(" NdisReadNetworkAddress(): ** not implemented yet ...\n");
}

VOID NdisMIndicateStatus(
  NDIS_HANDLE MiniportAdapterHandle,
  NDIS_STATUS GeneralStatus,
  PVOID StatusBuffer,
  UINT StatusBufferSize
)
{
}
