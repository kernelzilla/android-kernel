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

#ifndef _WINDOWS
	#include <sys/select.h>
	#include <unistd.h>
	#include <signal.h>

	#include "ipc.h"
	#include "g_tester.h"
	#include "wipp_ctrl.h"
#endif /* __LINUX__ */

#ifdef _WINDOWS
#else
	#include <errno.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


#include "ticon.h"
#include "console.h"
#include "cu_cmd.h"

static ConEntry_t *p_mon_root;
static ConEntry_t *p_cur_dir;
static char       *p_inbuf;
static volatile int stop_UI_Monitor;

#define INBUF_LENGTH          1024
/*#define PRINT_LEN_PER_PARM    40*/
#define ROOT_NAME             "/"

/* Internal functions */
static void        console_allocRoot( void );
static void        console_displayDir( ConEntry_t *p_dir );
static t_TokenType console_getWord( char *name, U16 len );
static t_TokenType console_getStrParam( char *buf, ConParm_t *param );
static t_TokenType console_analizeToken( char *name );
static U16         console_getNParms( ConEntry_t *p_token );
static int         console_parseParms( ConEntry_t *p_token, U16 *pnParms );
static ConEntry_t *console_searchToken( ConEntry_t *p_dir, char *name );
static void        console_dirHelp( void );
static void        console_displayHelp( ConEntry_t *p_token );
static int         console_chooseAlias( ConEntry_t *p_dir, ConEntry_t *p_new_token );


/***************************************************************

   Function : consoleRunScript

   Description: Execute command from file

   Parameters: script_file - name of script file

   Output:  !NULL - if 'quit' command was executed
***************************************************************/
int consoleRunScript( char *script_file )
{
    FILE *hfile = fopen(script_file, "r" );

    if( hfile )
    {
        char buf[INBUF_LENGTH];
        stop_UI_Monitor = FALSE;

        while( fgets(buf, sizeof(buf), hfile ) )
        {
            console_printf_terminal("script <%s>\n", script_file);
            console_ParseString( buf );
            if( stop_UI_Monitor )
                break;
        }

        fclose(hfile);
    }
    else
        perror( script_file );

    return stop_UI_Monitor;
}


/***************************************************************

   Function : consoleAddDirExt

   Description: Add subdirectory

   Parameters: p_root - root directory handle (might be NULL)
               name   - directory name

   Output:  the new created directory handle
            =NULL - failure
***************************************************************/
handle_t consoleAddDirExt(
                       handle_t   hRoot,          /* Upper directory handle. NULL=root */
                       const char *name,          /* New directory name */
                       const char *desc )         /* Optional dir description */
{
    ConEntry_t *p_root = (ConEntry_t *)hRoot;
    ConEntry_t *p_dir;
    ConEntry_t **p_e;

    if (!p_mon_root)
        console_allocRoot( );

    if (!p_root)
        p_root = p_mon_root;

    ASSERT( p_root && (p_root->sel == Dir) );

    if ( (p_dir=(ConEntry_t *)malloc( sizeof( ConEntry_t )) ) == NULL)
        return NULL;

    memset( p_dir, 0, sizeof( ConEntry_t ) );
    strncpy( p_dir->name, name, MAX_NAME_LEN );
    strncpy( p_dir->help, desc, MAX_HELP_LEN );
    p_dir->sel = Dir;

    console_chooseAlias( p_root, p_dir );

    /* Add new directory to the root's list */
    p_dir->u.dir.upper = p_root;
    p_e = &(p_root->u.dir.first);
    while (*p_e)
        p_e = &((*p_e)->next);
    *p_e = p_dir;

    return p_dir;
}

/***************************************************************

   Function : consoleAddToken

   Description: Add token

   Parameters: p_dir  - directory handle (might be NULL=root)
               name   - token name
               help   - help string
               p_func - token handler
               p_parms- array of parameter descriptions.
                        Must be terminated with {0}.
                        Each parm descriptor is a struct
                        { "myname",         - name
                          10,               - low value
                          20,               - high value
                          0 }               - default value =-1 no default
                                              or address for string parameter

   Output:  E_OK - OK
            !=0 - error
***************************************************************/
consoleErr consoleAddToken( handle_t      hDir,
                      const char    *name,
                      const char    *help,
                      FuncToken_t   p_func,
                      ConParm_t     p_parms[] )
{
   ConEntry_t *p_dir = (ConEntry_t *)hDir;
   ConEntry_t *p_token;
   ConEntry_t **p_e;
   U16       i;

   if (!p_mon_root)
      console_allocRoot( );

   if (!p_dir)
      p_dir = p_mon_root;

   ASSERT( p_dir && (p_dir->sel == Dir) );

   /* Initialize token structure */
   if ((p_token=(ConEntry_t *)calloc( 1, sizeof(ConEntry_t) )) == NULL)
   {
      fprintf(stderr, "** no memory **\n");
      return E_NOMEMORY;
   }


   /* Copy name */
   strncpy( p_token->name, name, MAX_NAME_LEN );
   strncpy( p_token->help, help, MAX_HELP_LEN );
   p_token->sel = Token;
   p_token->u.token.f_tokenFunc = p_func;

   /* Convert name to lower case and choose alias */
   console_chooseAlias( p_dir, p_token );

   /* Copy parameters */
   if ( p_parms )
   {
      for(i = 0; p_parms->name && p_parms->name[0] && ( i < MAX_NUM_OF_PARMS); i++ )
      {
         ConParm_t *p_token_parm = &p_token->u.token.parm[i];

         /* String parameter must have an address */
         if(p_parms->flags & (CON_PARM_STRING | CON_PARM_LINE))
         {
            if ( p_parms->hi_val >= INBUF_LENGTH )
         {
                fprintf(stderr, "** buffer too big: %s/%s\n", p_dir->name, name);
            free( p_token );
                return E_NOMEMORY;

            }
            if (p_parms->hi_val == 0 || (p_parms->flags & CON_PARM_RANGE) )
            {
            fprintf(stderr, "** Bad string param definition: %s/%s\n", p_dir->name, name );
                free( p_token );
            return E_BADPARM;
         }

            p_parms->value = (U32) calloc(1, p_parms->hi_val+1);
            if( !p_parms->value )
            {
                fprintf(stderr, "** No memory: %s/%s (max.size=%ld)\n", p_dir->name, name, p_parms->hi_val );
                free( p_token );
                return E_NOMEMORY;
            }
        }

         /* Copy parameter */
         *p_token_parm = *p_parms;
         if( p_token_parm->hi_val || p_token_parm->low_val )
             p_token_parm->flags |= CON_PARM_RANGE;
         p_token_parm->name = (const char *)p_token->u.token.name[i];
         strncpy( p_token->u.token.name[i], p_parms->name, MAX_NAME_LEN );
         ++p_parms;
      }
      if ((i == MAX_NUM_OF_PARMS) && p_parms->name[0])
      {
            fprintf(stderr, "** Too many params: %s/%s\n", p_dir->name, name );
         free( p_token );
         return E_TOOMANY;
      }
   }

   /* Add token to the directory */
   p_e = &(p_dir->u.dir.first);
   while (*p_e)
      p_e = &((*p_e)->next);
   *p_e = p_token;

   return E_OK;
}


/* Monitor driver */
void consoleStart( void )
{
#ifndef _WINDOWS
   fd_set read_set;
   int max_fd_index;
   int result;
   int pid;
#endif /* __LINUX__ */

   char inbuf[INBUF_LENGTH];

   if (!p_mon_root)
      return;

   stop_UI_Monitor = FALSE;
   console_displayDir( p_cur_dir );

   while(!stop_UI_Monitor)
   {
	   
#ifndef _WINDOWS
	   /***********************************************************************************/   
       /* Wait for one of two external events: 											 */
       /* -----------------------------------											*/
       /*																			   */
       /* 1. Data received from STDIN												  */
       /* 2. Data received from one of the TCP clients							 	 */
	   /* 3. Data received from iperf process stdout (if enabled)					*/
	   /****************************************************************************/   
	   
	   /* Prepare the read set fields */
	   FD_ZERO(&read_set);
	   FD_SET(0, &read_set);
	   FD_SET(ipc_pipe[0], &read_set);
	   FD_SET(wipp_control_general_process_out_pipe[0], &read_set);

	   /* Determine the maximum index of the file descriptor */
	   max_fd_index = (max(wipp_control_general_process_out_pipe[0], max(0, ipc_pipe[0])) + 1);
	   
	   /* Wait for event - blocking */
	   result = select(max_fd_index, &read_set, NULL, NULL, NULL);

	   if (result > 0)
	   {
		   if (FD_ISSET(0, &read_set))
		   {
			   /*****************************/
			   /* Data received from STDIN */
			   /***************************/
			   
                if ( fgets( inbuf, sizeof(inbuf), stdin ) <= 0 )
				 return;
  
				console_ParseString( inbuf );
			   
		   } 
		   
		   if (FD_ISSET(ipc_pipe[0], &read_set))
		   {
			   /**********************************/
			   /* Data received from TCP client */
			   /********************************/

			   result = read(ipc_pipe[0], (U8 *)inbuf, (U16)sizeof(inbuf));

			   /* Get the pid of the calling process */
			   pid = *(inbuf + 0) | (*(inbuf + 1) << 8);

			   /* Signal the calling process (tell him that we have 
			      received the command, and he can send us another one */
			   if (pid != 0xFFFF)
			   {
				   kill(pid, SIGUSR1); 
			   }
			   
			   if (result > 0)
			   {
				   console_ParseString(inbuf + 2);
			   }
		   }

		   if (FD_ISSET(wipp_control_general_process_out_pipe[0], &read_set))
		   {
			   /*****************************************/
			   /* Data received general process stdout */
			   /***************************************/

			   result = read(wipp_control_general_process_out_pipe[0], (U8 *)inbuf + 3, sizeof(inbuf) - 3);

			   if (result > 0)
			   {
				   wipp_control_send_iperf_results_to_host(WIPP_CONTROL_EVT_RUN_PROCESS_STDOUT, inbuf, result);
			   }
		   }
	   }
	   else
	   {
		   /* Error */
		   console_printf_terminal("Input selection mismatch...\n");

		   return;
	   }

#else  /* __LINUX__ */
#endif /* __LINUX__ */
   }
}


/* Parse the given input string and exit.
   All commands in the input string are executed one by one.
*/
void console_ParseString(char *input_string )
{
   ConEntry_t  *p_token;
   char        name[MAX_NAME_LEN];
   t_TokenType tType;
   U16        nParms;


#ifndef _WINDOWS
	/* Check if this is WIPP control command, if it is - process it */
	if (wipp_control_check_command(input_string))
	{
		return;
	}

	/* Check if this is g_tester control command, if it is - process it */
	if (g_tester_check_command((unsigned char*) input_string))
	{
		return;
	}
#endif /* __LINUX__ */

   if (!p_mon_root)
      return;

   if( input_string[strlen(input_string)-1] == '\n' )
   {
      char *s = (char *) &input_string[strlen(input_string)-1];
  	  *s = 0;
   }
   p_inbuf = (char *)input_string;
   stop_UI_Monitor = FALSE;

   /* Interpret empty string as "display directory" */
   if ( p_inbuf && !*p_inbuf )
      console_displayDir( p_cur_dir );

   while(!stop_UI_Monitor && p_inbuf && *p_inbuf)
   {
      tType = console_getWord( name, MAX_NAME_LEN );
      switch( tType )
      {

      case NameToken:
         p_token = console_searchToken( p_cur_dir, name );
         if (p_token == NULL)
         {
            fprintf( stderr, "**Error: '%s'**\n", name);
            p_inbuf = NULL;
         }
         else if (p_token->sel == Dir)
         {
            p_cur_dir = p_token;
            console_displayDir( p_cur_dir );
         }
         else
         {  /* Function token */
            if (!console_parseParms( p_token, &nParms ))
               console_displayHelp( p_token );
            else
               p_token->u.token.f_tokenFunc( p_token->u.token.parm, nParms );
         }
         break;

      case UpToken: /* Go to upper directory */
         if (p_cur_dir->u.dir.upper)
            p_cur_dir = p_cur_dir->u.dir.upper;
         console_displayDir( p_cur_dir );
         break;

      case RootToken: /* Go to the root directory */
         if (p_cur_dir->u.dir.upper)
            p_cur_dir = p_mon_root;
         console_displayDir( p_cur_dir );
         break;

      case HelpToken: /* Display help */
         if (( console_getWord( name, MAX_NAME_LEN ) == NameToken ) &&
             ((p_token = console_searchToken( p_cur_dir, name )) != NULL ) &&
             (p_token->sel == Token) )
            console_displayHelp( p_token );
         else
            console_dirHelp( );
         break;

      case DirHelpToken:
         console_displayDir( p_cur_dir );
			console_printf_terminal("Type ? <name> for command help, \"/\"-root, \"..\"-upper\n" );
         break;

      case BreakToken: /* Clear buffer */
         p_inbuf = NULL;
         break;

      case EmptyToken:
         break;

      }
   }
}


/* Stop monitor driver */
void consoleStop( void )
{
   stop_UI_Monitor = TRUE;
}


/*********************************************************/
/* Internal functions                                    */
/*********************************************************/

/* Allocate root directory */
void console_allocRoot( void )
{
   /* The very first call. Allocate root structure */
   if ((p_mon_root=(ConEntry_t *)calloc( 1, sizeof( ConEntry_t ) ) ) == NULL)
   {
      ASSERT( p_mon_root );
      return;
   }
   strcpy( p_mon_root->name, ROOT_NAME );
   p_mon_root->sel = Dir;
   p_cur_dir = p_mon_root;
}

/* Display directory */
void console_displayDir( ConEntry_t *p_dir )
{
   char out_buf[512];
   ConEntry_t *p_token;

   sprintf( out_buf, "%s%s> ", (p_dir==p_mon_root)? "" : ".../", p_dir->name );
   p_token = p_dir->u.dir.first;
   while( p_token )
   {
      if( (strlen(out_buf) + strlen(p_token->name) + 2)>= sizeof(out_buf) )
      {
          fprintf(stderr, "** console_displayDir(): buffer too small....\n");
          break;
      }
      strcat( out_buf, p_token->name );
      if ( p_token->sel == Dir )
         strcat( out_buf, "/" );
      p_token = p_token->next;
      if (p_token)
         strcat( out_buf, ", " );
   }
   console_printf_terminal("%s\n", out_buf );
}


/* Cut the first U16 from <p_inbuf>.
   Return the U16 in <name> and updated <p_inbuf>
*/
static t_TokenType console_getWord( char *name, U16 len )
{
   U16        i=0;
   t_TokenType tType;


   p_inbuf = console_ltrim(p_inbuf);

   while( *p_inbuf && *p_inbuf!=' ' && i<len )
      name[i++] = *(p_inbuf++);

   if (i<len)
      name[i] = 0;

   tType   = console_analizeToken( name );

   return tType;
}

static t_TokenType console_getStrParam( char *buf, ConParm_t *param )
{
    t_TokenType tType;
    U32         i, len = param->hi_val;
    char        *end_buf;

    p_inbuf = console_ltrim(p_inbuf);

    if( param->flags & CON_PARM_LINE )
    {
        strcpy(buf, p_inbuf );
        p_inbuf += strlen(p_inbuf);
    }
    else
    {
        if( *p_inbuf == '\"' )
        {
            end_buf = strchr(p_inbuf+1, '\"' );
            if( !end_buf )
            {
                fprintf(stderr, "** invalid string param: '%s'\n", p_inbuf );
                p_inbuf += strlen(p_inbuf);
                return EmptyToken;
            }
            if( (end_buf - p_inbuf - 1) > (int)len )
            {
                fprintf(stderr, "** param is too long: '%s'\n", p_inbuf );
                p_inbuf += strlen(p_inbuf);
                return EmptyToken;
            }
            *end_buf = 0;
            strcpy( buf, p_inbuf+1 );
            p_inbuf = end_buf + 1;
        }
        else
        {
            for( i=0; *p_inbuf && *p_inbuf!=' ' && i<len; i++ )
                buf[i] = *(p_inbuf++);

            buf[i] = 0;
            if( *p_inbuf && *p_inbuf != ' ' )
            {
                fprintf(stderr, "** param is too long: '%s'\n", p_inbuf-strlen(buf) );
                p_inbuf += strlen(p_inbuf);
                return EmptyToken;
            }
        }
    }

    tType   = console_analizeToken( buf );

    return tType;
}

/* Make a preliminary analizis of <name> token.
   Returns a token type (Empty, Up, Root, Break, Name)
*/
t_TokenType console_analizeToken( char *name )
{
   if (!name[0])
      return EmptyToken;

   if (!strcmp( name, TOKEN_UP ) )
      return UpToken;

   if (!strcmp( name, TOKEN_ROOT ) )
      return RootToken;

   if (!strcmp( name, TOKEN_BREAK ) )
      return BreakToken;

   if (!strcmp( name, TOKEN_HELP ) )
      return HelpToken;

   if (!strcmp( name, TOKEN_DIRHELP ) )
      return DirHelpToken;

   return NameToken;

}


/* Returns number of parameters of the given token
*/
static U16 console_getNParms( ConEntry_t *p_token )
{
   U16 i;
   if ( !p_token->u.token.parm )
      return 0;
   for( i=0;
        (i<MAX_NUM_OF_PARMS-1) &&
         p_token->u.token.parm[i].name &&
         p_token->u.token.parm[i].name[0];
        i++ )
      ;
   return i;
}

/* Parse p_inbuf string based on parameter descriptions in <p_token>.
   Fill parameter values in <p_token>.
   Returns the number of parameters filled.
   To Do: add a option of one-by-one user input of missing parameters.
*/
int console_parseParms( ConEntry_t *p_token, U16 *pnParms )
{
    U16 nTotalParms = console_getNParms( p_token );
    U16 nParms=0;
    char *end_buf = NULL, parm[INBUF_LENGTH];
    U16 i, print_params = 0;
    U32 val = 0;
    S32 sval = 0;

    /* Mark all parameters as don't having an explicit value */
    for( i=0; i<nTotalParms; i++ )
            p_token->u.token.parm[i].flags |= CON_PARM_NOVAL;

    /*        -----------------              */
    p_inbuf = console_ltrim(p_inbuf);
    if( p_inbuf[0] == '!' && p_inbuf[1] == '!' )
    {
        p_inbuf += 2; print_params = 1;
    }
    /*        -----------------              */

    /* Build a format string */
    for( i=0; i<nTotalParms; i++ )
    {
        if (p_token->u.token.parm[i].flags & (CON_PARM_STRING | CON_PARM_LINE) )
        {
            /* For a string parameter value is the string address */
            /* and hi_val is the string length                   */
            if (console_getStrParam( parm, &p_token->u.token.parm[i] ) != NameToken)
                break;
            if( strlen(parm) > p_token->u.token.parm[i].hi_val ||
                (p_token->u.token.parm[i].low_val && p_token->u.token.parm[i].low_val > strlen(parm) ) )
        {
                fprintf(stderr, "param '%s' must be %ld..%ld chars\n", p_token->u.token.parm[i].name,
                        p_token->u.token.parm[i].low_val, p_token->u.token.parm[i].hi_val);
                return FALSE;

            }
            strcpy( (char *)p_token->u.token.parm[i].value, parm );
        }
        else
        {
            if (console_getWord( parm, MAX_PARM_LEN ) != NameToken)
                break;

            if (p_token->u.token.parm[i].flags & CON_PARM_SIGN)
                sval = strtol( parm, &end_buf, 0 );
            else
                val = strtoul( parm, &end_buf, 0 );
            if( /*errno || */end_buf <= parm )
                    break;

/*             if (sscanf( parm, "%i", &val ) != 1)*/
/*                 break;*/
            
            /* Check value */
            if (p_token->u.token.parm[i].flags & CON_PARM_RANGE)
            {           
                if (p_token->u.token.parm[i].flags & CON_PARM_SIGN)
                {
                    if ((sval < (S32)p_token->u.token.parm[i].low_val) ||
                        (sval > (S32)p_token->u.token.parm[i].hi_val) )
                    {
                        fprintf( stderr, "%s: %d out of range (%d, %d)\n",
                            p_token->u.token.parm[i].name, (int)sval,
                            (int)p_token->u.token.parm[i].low_val, (int)p_token->u.token.parm[i].hi_val );
                        return FALSE;
                    }

                }
                else
                {                    
                    if ((val < p_token->u.token.parm[i].low_val) ||
                        (val > p_token->u.token.parm[i].hi_val) )
                    {
                        fprintf( stderr, "%s: %ld out of range (%ld, %ld)\n",
                            p_token->u.token.parm[i].name, val,
                            p_token->u.token.parm[i].low_val, p_token->u.token.parm[i].hi_val );
                        return FALSE;
                    }
                }
            }

            if (p_token->u.token.parm[i].flags & CON_PARM_SIGN)                
                p_token->u.token.parm[i].value = sval;
            else
                p_token->u.token.parm[i].value = val;
        }

        p_token->u.token.parm[i].flags &= ~CON_PARM_NOVAL;
        ++nParms;
    }

    /* Process default values */
    for( ; i<nTotalParms; i++ )
    {
        if ((p_token->u.token.parm[i].flags & CON_PARM_DEFVAL) != 0)
        {
            p_token->u.token.parm[i].flags &= ~CON_PARM_NOVAL;
            ++nParms;
        }
        else if (!(p_token->u.token.parm[i].flags & CON_PARM_OPTIONAL) )
        {
            /* Mandatory parameter missing */
            return FALSE;
        }
    }

    if( print_params )
    {
        printf("Params: %d\n", nParms );
        for (i=0; i<nParms; i++ )
        {
            console_printf_terminal("%d: %s - flags:%d", 
                i+1, p_token->u.token.parm[i].name,
                p_token->u.token.parm[i].flags);
            
            if (p_token->u.token.parm[i].flags & CON_PARM_SIGN)  
                console_printf_terminal("min:%d, max:%d, value:%d ",p_token->u.token.parm[i].low_val, p_token->u.token.parm[i].hi_val,
                    p_token->u.token.parm[i].value);
            else
                console_printf_terminal("min:%ld, max:%ld, value:%ld ",p_token->u.token.parm[i].low_val, p_token->u.token.parm[i].hi_val,
                    p_token->u.token.parm[i].value);
            
            console_printf_terminal("(%#lx)",p_token->u.token.parm[i].value );
            
            if( p_token->u.token.parm[i].flags & (CON_PARM_LINE | CON_PARM_STRING ))
            {
                printf(" - '%s'", (char *) p_token->u.token.parm[i].value );
            }
            printf("\n");
        }

    }
    *pnParms = nParms;

    return TRUE;
}

/* Serach a token by name in the current directory */
ConEntry_t *console_searchToken( ConEntry_t *p_dir, char *name )
{
   ConEntry_t *p_token;
   U16        name_len = (U16)strlen( name );

   /* Check alias */
   p_token = p_dir->u.dir.first;
   while( p_token )
   {
      if (p_token->alias &&
          (name_len == ALIAS_LEN) &&
          !console_stricmp( p_token->alias, name, ALIAS_LEN ) )
          return p_token;
      p_token = p_token->next;
   }

   /* Check name */
   p_token = p_dir->u.dir.first;
   while( p_token )
   {
      if (!console_stricmp( p_token->name, name, name_len ) )
         break;
      p_token = p_token->next;
   }

   return p_token;
}


/* Display help for each entry in the current directory */
void  console_dirHelp( void )
{
   ConEntry_t *p_token;
   char        print_str[80];

   p_token = p_cur_dir->u.dir.first;

   while( p_token )
   {
      if (p_token->sel == Dir)
         sprintf( print_str, "%s: directory\n", p_token->name );
      else
         sprintf( print_str, "%s(%d parms): %s\n",
                  p_token->name, console_getNParms(p_token), p_token->help );
      console_printf_terminal( print_str );
      p_token = p_token->next;
   }

   console_printf_terminal( "Type ? <name> for command help, \"/\"-root, \"..\"-upper\n" );
}


/* Display help a token */
void  console_displayHelp( ConEntry_t *p_token )
{
   char bra, ket;
   U16 nTotalParms = console_getNParms( p_token );
   U16 i;

   
   console_printf_terminal( "%s: %s ", p_token->help, p_token->name );
   for( i=0; i<nTotalParms; i++ )
   {
      if (p_token->u.token.parm[i].flags & CON_PARM_OPTIONAL)
      {
         bra = '['; ket=']';
      }
      else
      {
         bra = '<'; ket='>';
      }
      console_printf_terminal( "%c%s", bra, p_token->u.token.parm[i].name );
      if (p_token->u.token.parm[i].flags & CON_PARM_DEFVAL)
      {
          console_printf_terminal("=%lu", p_token->u.token.parm[i].value);
      }
      if (p_token->u.token.parm[i].flags & CON_PARM_RANGE)
      {
          console_printf_terminal( (p_token->u.token.parm[i].flags & CON_PARM_SIGN) ? " (%d..%d%s)" : " (%lu..%lu%s)",
                  p_token->u.token.parm[i].low_val,
                  p_token->u.token.parm[i].hi_val,
                  (p_token->u.token.parm[i].flags & (CON_PARM_STRING | CON_PARM_LINE)) ? " chars" : "" );
          
      }
      console_printf_terminal( "%c \n",ket );
   }
}

/* Choose unique alias for <name> in <p_dir> */
/* Currently only single-character aliases are supported */
int console_chooseAlias( ConEntry_t *p_dir, ConEntry_t *p_new_token )
{
   ConEntry_t *p_token;
   int         i;
   char        c;
   char *new_alias = NULL;

   /* find alias given from user */
   for(i=0; p_new_token->name[i]; i++ )
   {
       if( isupper(p_new_token->name[i]) )
       {
           new_alias = &p_new_token->name[i];
           break;
       }
   }

   console_strlwr( p_new_token->name );

   if( new_alias )
   {
      p_token = p_dir->u.dir.first;

      while( p_token )
      {
         if (p_token->alias && (tolower( *p_token->alias ) == *new_alias) )
         {
/*            *new_alias = toupper(*new_alias);*/
            fprintf( stderr, "**Error: duplicated alias '%c' in <%s> and <%s>**\n", *new_alias,
                    p_token->name, p_new_token->name );
            return 0;
         }
         p_token = p_token->next;
      }
      *new_alias = toupper(*new_alias);
      p_new_token->alias = new_alias;
      return 1;
   }

   i = 0;
   while( p_new_token->name[i] )
   {
      c = p_new_token->name[i];
      p_token = p_dir->u.dir.first;

      while( p_token )
      {
         if (p_token->alias &&
             (tolower( *p_token->alias ) == c) )
            break;
         p_token = p_token->next;
      }
      if (p_token)
         ++i;
      else
      {
         p_new_token->name[i] = toupper( c );
         p_new_token->alias   = &p_new_token->name[i];
         break;
      }
   }
   return 1;
}


/* Convert string s to lower case. Return pointer to s */
char  * console_strlwr( char *s )
{
   char  *s0=s;

   while( *s )
   {
      *s = tolower( *s );
      ++s;
   }

   return s0;
}


/* Compare strings case insensitive */
int console_stricmp( char *s1, char *s2, U16 len )
{
   int  i;

   for( i=0; i<len && s1[i] && s2[i]; i++ )
   {
      if (tolower( s1[i])  != tolower( s2[i] ))
         break;
   }

   return ( (len - i) * (s1[i] - s2[i]) );
}

/* Remove leading blanks */
char * console_ltrim(char *s )
{
    while( *s == ' ' || *s == '\t' ) s++;
    return s;
}

#ifdef _WINDOWS
#endif /* _WINDOWS*/


