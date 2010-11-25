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

#ifndef tiwlan_console_h
#define tiwlan_console_h

#include <stdio.h>
#include <assert.h>
/*
 * --------------------- *
 *     error codes
 * --------------------- *
 */
typedef enum
{
   E_OK = 0
   , E_BADPARM
   , E_TOOMANY
   , E_NOMEMORY
   , E_NOT_FOUND
   , E_EXISTS
   , E_DUMMY
} consoleErr_t;

typedef consoleErr_t  consoleErr;

typedef void *                   handle_t;

typedef unsigned char            U8;
typedef signed char              S8;
typedef unsigned short           U16;
typedef signed short             S16;
typedef unsigned long            U32;
typedef signed long              S32;



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Monitor parameter flags */
#define CON_PARM_OPTIONAL    0x01  /* Parameter is optional */
#define CON_PARM_DEFVAL      0x02  /* Default value is set */
#define CON_PARM_RANGE       0x04  /* Range is set */
#define CON_PARM_STRING      0x08  /* String parm */
#define CON_PARM_LINE        0x10  /* String from the current parser position till EOL */
#define CON_PARM_SIGN        0x20  /* Signed param */
#define CON_PARM_NOVAL       0x80  /* Internal flag: parameter is anassigned */

/* Function parameter structure */
typedef struct ConParm_t
{
   const char *name;             /* Parameter name. Shouldn't be allocated on stack! */
   U8 flags;                     /* Combination of CON_PARM_??? flags */
   U32 low_val;                  /* Low val for range checking */
   U32 hi_val;                   /* Hi val for range checking/max length of string */
   U32 value;                    /* Value/address of string parameter */
} ConParm_t;

#define CON_LAST_PARM       { NULL, 0, 0, 0, 0 }

/* Monitor command handler prototype */
typedef void (*FuncToken_t)(ConParm_t parm[], U16 nParms);

/* Add subdirectory to the p_root directory
   Returns the new directory handle
*/
handle_t consoleAddDirExt(
                       handle_t   hRoot,          /* Upper directory handle. NULL=root */
                       const char *name,          /* New directory name */
                       const char *desc );    /* Optional directory description */

/* Add token */
consoleErr consoleAddToken( handle_t     hDir,     /* Directory handle. NULL=root */
                      const char    *name,    /* Token name. Shouldn't be allocated on stack! */
                      const char    *help,    /* Token help. Shouldn't be allocated on stack! */
                      FuncToken_t   p_func,   /* Token handler */
                      ConParm_t     p_parms[]);/* Array of token parameters. */
                                              /* The last array element has parameter */
                                              /* name = NULL */

/* Monitor driver.
   Calls XX_Gets in infinite loop to get input string.
   Gives the string to console_ParseString for processing.
   Monitor token handler can call consoleStop() to exit the
   consoleStart.
*/
void consoleStart( void );

/* Parse the given input string and exit.
   All commands in the input string are executed one by one.
*/
void console_ParseString( char *input_string );

/* Stop monitor driver */
void consoleStop( void );

/* Execute commands from 'script_file' */
int consoleRunScript( char *script_file );

#ifdef _WINDOWS
#endif

#ifdef __cplusplus
}
#endif

/* ----------------------------------------------------- */

#define ALIAS_LEN                1

#define MAX_NAME_LEN       80
#define MAX_HELP_LEN       80
#define MAX_PARM_LEN       20
#define MAX_NUM_OF_PARMS   30

#define TOKEN_UP           ".."
#define TOKEN_ROOT         "/"
#define TOKEN_BREAK        "#"
#define TOKEN_HELP         "?"
#define TOKEN_DIRHELP      "help"

#ifndef FALSE
	#define FALSE    0
#endif

#ifndef TRUE
	#define TRUE    1
#endif

#ifndef __LINUX__ // TRS:WDK
	#define perror(str) printf("\nError at %s:%d  - %s.\n", __FILE__, __LINE__, (str))
#endif /* __LINUX__ */
//TRS end
#ifdef    __cplusplus
	extern "C" {
#endif   /* __cplusplus */

#ifndef _WINDOWS /* TRS:WDK __LINUX__ */
	#ifdef ERRCHK
		# define ASSERT(p) assert(p)
	#else
		# define ASSERT(p) do {} while (0)
	#endif 
#endif /* TRS:WDK __LINUX__ */

#ifdef __cplusplus
}
#endif

typedef enum { Dir, Token } ConEntry_type_t;

/* Parameter name and format */
typedef char (ParmName_t)[MAX_NAME_LEN+1];

/* Monitor token structure */
typedef struct ConEntry_t
{
   struct ConEntry_t   *next;
   char                name[MAX_NAME_LEN+1];    /* Entry name */
   char                help[MAX_HELP_LEN+1];    /* Help string */
   char                *alias;                  /* Alias - always in upper case*/
   ConEntry_type_t     sel;                   /* Entry selector */

   union {
      struct
      {
         struct ConEntry_t   *upper;            /* Upper directory */
         struct ConEntry_t   *first;            /* First entry */
      } dir;
      struct t_Token
      {
         FuncToken_t    f_tokenFunc;            /* Token handler */
         ConParm_t      parm[MAX_NUM_OF_PARMS]; /* Parameters array */
         ParmName_t     name[MAX_NUM_OF_PARMS]; /* Parameter name */
      } token;
   } u;
} ConEntry_t;

/* Token types */
typedef enum
{
   EmptyToken,
   UpToken,
   RootToken,
   BreakToken,
   HelpToken,
   DirHelpToken,
   NameToken
} t_TokenType;

char * console_strlwr( char *s );
int    console_stricmp( char *s1, char *s2, U16 len );
char * console_ltrim( char *s );

#endif /* #ifndef tiwlan_console_h */
