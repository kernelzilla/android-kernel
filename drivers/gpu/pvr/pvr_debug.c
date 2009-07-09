/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

  
#ifndef AUTOCONF_INCLUDED
 #include <linux/config.h>
#endif

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/tty.h>			
#include <stdarg.h>
#include "img_types.h"
#include "pvr_debug.h"
#include "proc.h"

#if defined(DEBUG) || defined(TIMING)

IMG_UINT32	gPVRDebugLevel = DBGPRIV_WARNING;

#define PVR_STRING_TERMINATOR		'\0'
#define PVR_IS_FILE_SEPARATOR(character) ( ((character) == '\\') || ((character) == '/') )

IMG_VOID PVRSRVDebugPrintf	(
						IMG_UINT32	ui32DebugLevel,
						const IMG_CHAR*	pszFileName,
						IMG_UINT32	ui32Line,
						const IMG_CHAR*	pszFormat,
						...
					)
{
	IMG_BOOL bTrace, bDebug;
#if !defined(__sh__)
	IMG_CHAR *pszLeafName;
	
	pszLeafName = (IMG_CHAR *)strrchr (pszFileName, '\\');
	
	if (pszLeafName)
	{
		pszFileName = pszLeafName;
	}
#endif 
		
	bTrace = gPVRDebugLevel & ui32DebugLevel & DBGPRIV_CALLTRACE;
	bDebug = ((gPVRDebugLevel & DBGPRIV_ALLLEVELS) >= ui32DebugLevel);

	if (bTrace || bDebug)
	{
		va_list vaArgs;
		static IMG_CHAR szBuffer[256];

		va_start (vaArgs, pszFormat);

		
		if (bDebug)
		{
			switch(ui32DebugLevel)
			{
				case DBGPRIV_FATAL:
				{
					strcpy (szBuffer, "PVR_K:(Fatal): ");
					break;
				}
				case DBGPRIV_ERROR:
				{
					strcpy (szBuffer, "PVR_K:(Error): ");
					break;
				}
				case DBGPRIV_WARNING:
				{
					strcpy (szBuffer, "PVR_K:(Warning): ");
					break;
				}
				case DBGPRIV_MESSAGE:
				{
					strcpy (szBuffer, "PVR_K:(Message): ");
					break;
				}
				case DBGPRIV_VERBOSE:
				{
					strcpy (szBuffer, "PVR_K:(Verbose): ");
					break;
				}
				default:
				{
					strcpy (szBuffer, "PVR_K:(Unknown message level)");
					break;
				}
			}
		}
		else
		{
			strcpy (szBuffer, "PVR_K: ");
		}

		vsprintf (&szBuffer[strlen(szBuffer)], pszFormat, vaArgs);

 		

 		if (!bTrace)
		{
			sprintf (&szBuffer[strlen(szBuffer)], " [%d, %s]", (IMG_INT)ui32Line, pszFileName);
		}

		printk(KERN_INFO "%s\n", szBuffer);

		va_end (vaArgs);
	}
}

IMG_VOID PVRSRVDebugAssertFail(const IMG_CHAR* pszFile, IMG_UINT32 uLine)
{
	PVRSRVDebugPrintf(DBGPRIV_FATAL, pszFile, uLine, "Debug assertion failed!");
	BUG();
}

IMG_VOID PVRSRVTrace(const IMG_CHAR* pszFormat, ...)
{
	static IMG_CHAR szMessage[PVR_MAX_DEBUG_MESSAGE_LEN+1];
	IMG_CHAR* pszEndOfMessage = IMG_NULL;
	va_list ArgList;

	strncpy(szMessage, "PVR: ", PVR_MAX_DEBUG_MESSAGE_LEN);

	pszEndOfMessage = &szMessage[strlen(szMessage)];

	va_start(ArgList, pszFormat);
	vsprintf(pszEndOfMessage, pszFormat, ArgList);
	va_end(ArgList);

	strcat(szMessage,"\n");

	printk(KERN_INFO "%s", szMessage);
}


IMG_VOID PVRDebugSetLevel(IMG_UINT32 uDebugLevel)
{
	printk(KERN_INFO "PVR: Setting Debug Level = 0x%x\n",(IMG_UINT)uDebugLevel);

	gPVRDebugLevel = uDebugLevel;
}

IMG_INT PVRDebugProcSetLevel(struct file *file, const IMG_CHAR *buffer, IMG_UINT32 count, IMG_VOID *data)
{
#define	_PROC_SET_BUFFER_SZ		2
	IMG_CHAR data_buffer[_PROC_SET_BUFFER_SZ];

	if (count != _PROC_SET_BUFFER_SZ)
	{
		return -EINVAL;
	}
	else
	{
		if (copy_from_user(data_buffer, buffer, count))
			return -EINVAL;
		if (data_buffer[count - 1] != '\n')
			return -EINVAL;
		PVRDebugSetLevel(data_buffer[0] - '0');
	}
	return (count);
}

IMG_INT PVRDebugProcGetLevel(IMG_CHAR *page, IMG_CHAR **start, off_t off, IMG_INT count, IMG_INT *eof, IMG_VOID *data)
{
	if (off == 0) {
		*start = (IMG_CHAR *)1;
		return printAppend(page, count, 0, "%lu\n", gPVRDebugLevel);
	}
	*eof = 1;
	return 0;
}

#endif 
