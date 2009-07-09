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

#ifndef __SERVICES_PROC_H__
#define __SERVICES_PROC_H__

#include <asm/system.h>		
#include <linux/proc_fs.h>	

#define END_OF_FILE (off_t) -1

typedef off_t (pvr_read_proc_t)(IMG_CHAR *, size_t, off_t);

off_t printAppend(IMG_CHAR * buffer, size_t size, off_t off, const IMG_CHAR * format, ...)
	__attribute__((format(printf, 4, 5)));

IMG_INT CreateProcEntries(IMG_VOID);

IMG_INT CreateProcReadEntry (const IMG_CHAR * name, pvr_read_proc_t handler);

IMG_INT CreateProcEntry(const IMG_CHAR * name, read_proc_t rhandler, write_proc_t whandler, IMG_VOID *data);

IMG_INT CreatePerProcessProcEntry(const IMG_CHAR * name, read_proc_t rhandler, write_proc_t whandler, IMG_VOID *data);

IMG_VOID RemoveProcEntry(const IMG_CHAR * name);

IMG_VOID RemovePerProcessProcEntry(const IMG_CHAR * name);

IMG_VOID RemoveProcEntries(IMG_VOID);

#endif
