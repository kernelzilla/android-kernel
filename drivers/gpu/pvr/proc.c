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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#include "services_headers.h"

#include "queue.h"
#include "resman.h"
#include "pvrmmap.h"
#include "pvr_debug.h"
#include "pvrversion.h"
#include "proc.h"
#include "perproc.h"
#include "env_perproc.h"

#ifdef DEBUG
IMG_INT PVRDebugProcSetLevel(struct file *file, const IMG_CHAR *buffer, IMG_UINT32 count, IMG_VOID *data);
IMG_INT PVRDebugProcGetLevel(IMG_CHAR *page, IMG_CHAR **start, off_t off, IMG_INT count, IMG_INT *eof, IMG_VOID *data);

#ifdef PVR_MANUAL_POWER_CONTROL
IMG_INT PVRProcSetPowerLevel(struct file *file, const IMG_CHAR *buffer, IMG_UINT32 count, IMG_VOID *data);
IMG_INT PVRProcGetPowerLevel(IMG_CHAR *page, IMG_CHAR **start, off_t off, IMG_INT count, IMG_INT *eof, IMG_VOID *data);
#endif
#endif

static struct proc_dir_entry * dir;

static off_t procDumpSysNodes(IMG_CHAR *buf, size_t size, off_t off);
static off_t procDumpVersion(IMG_CHAR *buf, size_t size, off_t off);

static const IMG_CHAR PVRProcDirRoot[] = "pvr";

off_t printAppend(IMG_CHAR * buffer, size_t size, off_t off, const IMG_CHAR * format, ...)
{
    IMG_INT n;
    IMG_INT space = size - off;
    va_list ap;

    PVR_ASSERT(space >= 0);

    va_start (ap, format);

    n = vsnprintf (buffer+off, space, format, ap);

    va_end (ap);
    
    if (n >= space || n < 0)
    {
	
        buffer[size - 1] = 0;
        return size - 1;
    }
    else
    {
        return off + n;
    }
}


static IMG_INT pvr_read_proc(IMG_CHAR *page, IMG_CHAR **start, off_t off,
                         IMG_INT count, IMG_INT *eof, IMG_VOID *data)
{
	pvr_read_proc_t *pprn = data;

    off_t len = pprn (page, count, off);

    if (len == END_OF_FILE)
    {
        len  = 0;
        *eof = 1;
    }
    else if (!len)             
    {
        *start = (IMG_CHAR *) 0;   
    }
    else
    {
        *start = (IMG_CHAR *) 1;
    }

    return len;
}


static IMG_INT CreateProcEntryInDir(struct proc_dir_entry *pdir, const IMG_CHAR * name, read_proc_t rhandler, write_proc_t whandler, IMG_VOID *data)
{
    struct proc_dir_entry * file;
    mode_t mode;

    if (!pdir)
    {
        PVR_DPF((PVR_DBG_ERROR, "CreateProcEntryInDir: parent directory doesn't exist"));

        return -ENOMEM;
    }

    mode = S_IFREG;

    if (rhandler)
    {
	mode |= S_IRUGO;
    }

    if (whandler)
    {
	mode |= S_IWUSR;
    }

    file = create_proc_entry(name, mode, pdir);

    if (file)
    {
        file->owner = THIS_MODULE;
		file->read_proc = rhandler;
		file->write_proc = whandler;
		file->data = data;

		PVR_DPF((PVR_DBG_MESSAGE, "Created proc entry %s in %s", name, pdir->name));

        return 0;
    }

    PVR_DPF((PVR_DBG_ERROR, "CreateProcEntry: cannot create proc entry %s in %s", name, pdir->name));

    return -ENOMEM;
}


IMG_INT CreateProcEntry(const IMG_CHAR * name, read_proc_t rhandler, write_proc_t whandler, IMG_VOID *data)
{
    return CreateProcEntryInDir(dir, name, rhandler, whandler, data);
}


IMG_INT CreatePerProcessProcEntry(const IMG_CHAR * name, read_proc_t rhandler, write_proc_t whandler, IMG_VOID *data)
{
    PVRSRV_ENV_PER_PROCESS_DATA *psPerProc;
    IMG_UINT32 ui32PID;

    if (!dir)
    {
        PVR_DPF((PVR_DBG_ERROR, "CreatePerProcessProcEntries: /proc/%s doesn't exist", PVRProcDirRoot));

        return -ENOMEM;
    }

    ui32PID = OSGetCurrentProcessIDKM();

    psPerProc = PVRSRVPerProcessPrivateData(ui32PID);
    if (!psPerProc)
    {
        PVR_DPF((PVR_DBG_ERROR, "CreatePerProcessProcEntries: no per process data"));

        return -ENOMEM;
    }

    if (!psPerProc->psProcDir)
    {
        IMG_CHAR dirname[16];
        IMG_INT ret;

        ret = snprintf(dirname, sizeof(dirname), "%lu", ui32PID);

        if (ret <=0 || ret >= sizeof(dirname))
	{
		PVR_DPF((PVR_DBG_ERROR, "CreatePerProcessProcEntries: couldn't generate per process proc directory name \"%u\"", ui32PID));

                return -ENOMEM;
	}
	else
        {
            psPerProc->psProcDir = proc_mkdir(dirname, dir);
            if (!psPerProc->psProcDir)
	    {
                PVR_DPF((PVR_DBG_ERROR, "CreatePerProcessProcEntries: couldn't create per process proc directory /proc/%s/%u", PVRProcDirRoot, ui32PID));

                return -ENOMEM;
	    }
        }
    }

    return CreateProcEntryInDir(psPerProc->psProcDir, name, rhandler, whandler, data);
}


IMG_INT CreateProcReadEntry(const IMG_CHAR * name, pvr_read_proc_t handler)
{
    struct proc_dir_entry * file;

    if (!dir)
    {
        PVR_DPF((PVR_DBG_ERROR, "CreateProcReadEntry: cannot make proc entry /proc/%s/%s: no parent", PVRProcDirRoot, name));

        return -ENOMEM;
    }

    file = create_proc_read_entry (name, S_IFREG | S_IRUGO, dir, pvr_read_proc, (IMG_VOID *)handler);

    if (file)
    {
        file->owner = THIS_MODULE;

        return 0;
    }

    PVR_DPF((PVR_DBG_ERROR, "CreateProcReadEntry: cannot make proc entry /proc/%s/%s: no memory", PVRProcDirRoot, name));

    return -ENOMEM;
}


IMG_INT CreateProcEntries(IMG_VOID)
{
    dir = proc_mkdir (PVRProcDirRoot, NULL);

    if (!dir)
    {
        PVR_DPF((PVR_DBG_ERROR, "CreateProcEntries: cannot make /proc/%s directory", PVRProcDirRoot));

        return -ENOMEM;
    }

    if (CreateProcReadEntry("queue", QueuePrintQueues) ||
		CreateProcReadEntry("version", procDumpVersion) ||
		CreateProcReadEntry("nodes", procDumpSysNodes))
    {
        PVR_DPF((PVR_DBG_ERROR, "CreateProcEntries: couldn't make /proc/%s files", PVRProcDirRoot));

        return -ENOMEM;
    }

#ifdef DEBUG
	if (CreateProcEntry ("debug_level", PVRDebugProcGetLevel, PVRDebugProcSetLevel, 0))
    {
        PVR_DPF((PVR_DBG_ERROR, "CreateProcEntries: couldn't make /proc/%s/debug_level", PVRProcDirRoot));

        return -ENOMEM;
    }

#ifdef PVR_MANUAL_POWER_CONTROL
	if (CreateProcEntry("power_control", PVRProcGetPowerLevel, PVRProcSetPowerLevel, 0))
    {
        PVR_DPF((PVR_DBG_ERROR, "CreateProcEntries: couldn't make /proc/%s/power_control", PVRProcDirRoot));

        return -ENOMEM;
    }
#endif
#endif

    return 0;
}


IMG_VOID RemoveProcEntry(const IMG_CHAR *name)
{
    if (dir)
    {
        remove_proc_entry(name, dir);
        PVR_DPF((PVR_DBG_MESSAGE, "Removing /proc/%s/%s", PVRProcDirRoot, name));
    }
}


IMG_VOID RemovePerProcessProcEntry(const IMG_CHAR *name)
{
    PVRSRV_ENV_PER_PROCESS_DATA *psPerProc = PVRSRVFindPerProcessPrivateData();

    if (!psPerProc)
    {
	PVR_DPF((PVR_DBG_ERROR, "CreatePerProcessProcEntries: can't remove %s, no per process data", name));
	return;
    }

    if (psPerProc->psProcDir)
    {
	remove_proc_entry(name, psPerProc->psProcDir);

	PVR_DPF((PVR_DBG_MESSAGE, "Removing proc entry %s from %s", name, psPerProc->psProcDir->name));
    }
}


IMG_VOID RemovePerProcessProcDir(PVRSRV_ENV_PER_PROCESS_DATA *psPerProc)
{
    if (psPerProc->psProcDir)
    {
        while (psPerProc->psProcDir->subdir)
        {
            PVR_DPF((PVR_DBG_WARNING, "Belatedly removing /proc/%s/%s/%s", PVRProcDirRoot, psPerProc->psProcDir->name, psPerProc->psProcDir->subdir->name));

            RemoveProcEntry(psPerProc->psProcDir->subdir->name);
        }
        RemoveProcEntry(psPerProc->psProcDir->name);
    }
}

IMG_VOID RemoveProcEntries(IMG_VOID)
{
#ifdef DEBUG
    RemoveProcEntry("debug_level");
#ifdef PVR_MANUAL_POWER_CONTROL
    RemoveProcEntry("power_control");
#endif
#endif
    RemoveProcEntry("queue");
    RemoveProcEntry("nodes");
    RemoveProcEntry("version");

    while (dir->subdir)
    {
	PVR_DPF((PVR_DBG_WARNING, "Belatedly removing /proc/%s/%s", PVRProcDirRoot, dir->subdir->name));

	RemoveProcEntry(dir->subdir->name);
    }

    remove_proc_entry(PVRProcDirRoot, NULL);
}


static off_t procDumpVersion(IMG_CHAR *buf, size_t size, off_t off)
{
    SYS_DATA *psSysData;
    
    if (off == 0)
    {
	return printAppend(buf, size, 0,
						"Version %s (%s) %s\n",
						PVRVERSION_STRING,
						PVR_BUILD_TYPE, PVR_BUILD_DIR);
    }

    if (SysAcquireData(&psSysData) != PVRSRV_OK)
    {
	return PVRSRV_ERROR_GENERIC;
    }
    
    if (off == 1)
    {
        IMG_CHAR *pszSystemVersionString = "None";

        if(psSysData->pszVersionString)
        {
            pszSystemVersionString = psSysData->pszVersionString;
        }
            
        if(strlen(pszSystemVersionString) 
            + strlen("System Version String: \n") 
            + 1 > size)
        {
            return 0;
        }
        return printAppend(buf, size, 0,
                            "System Version String: %s\n",
                            pszSystemVersionString);
    }
    
    return END_OF_FILE;
}


static const IMG_CHAR *deviceTypeToString(PVRSRV_DEVICE_TYPE deviceType)
{
    switch (deviceType)
    {
        default:
        {
            static IMG_CHAR text[10];

            sprintf(text, "?%x", deviceType);

            return text;
        }
    }
}


static const IMG_CHAR *deviceClassToString(PVRSRV_DEVICE_CLASS deviceClass)
{
    switch (deviceClass) 
    {
	case PVRSRV_DEVICE_CLASS_3D:
	{
	    return "3D";
	}
	case PVRSRV_DEVICE_CLASS_DISPLAY:
	{
	    return "display";
	}
	case PVRSRV_DEVICE_CLASS_BUFFER:
	{
	    return "buffer";
	}
	default:
	{
	    static IMG_CHAR text[10];

	    sprintf(text, "?%x", deviceClass);
	    return text;
	}
    }
}

static
off_t procDumpSysNodes(IMG_CHAR *buf, size_t size, off_t off)
{
    SYS_DATA 			*psSysData;
    PVRSRV_DEVICE_NODE	*psDevNode;
    off_t				len;

    
    if (size < 80)
    {
	return 0;
    }

    if (off == 0)
    {
	return printAppend(buf, size, 0, 
						"Registered nodes\n"
						"Addr     Type     Class    Index Ref pvDev     Size Res\n");
    }

    if (SysAcquireData(&psSysData) != PVRSRV_OK)
    {
	return PVRSRV_ERROR_GENERIC;
    }

    
    for(psDevNode = psSysData->psDeviceNodeList;
		--off && psDevNode;
		psDevNode = psDevNode->psNext)
	;

    if (!psDevNode)
    {
	return END_OF_FILE;
    }

    len = printAppend(buf, size, 0,
				  "%p %-8s %-8s %4d  %2lu  %p  %3lu  %p\n",
				  psDevNode,
				  deviceTypeToString(psDevNode->sDevId.eDeviceType),
				  deviceClassToString(psDevNode->sDevId.eDeviceClass),
				  psDevNode->sDevId.eDeviceClass,
				  psDevNode->ui32RefCount,
				  psDevNode->pvDevice,
				  psDevNode->ui32pvDeviceSize,
				  psDevNode->hResManContext);
    return (len);
}

