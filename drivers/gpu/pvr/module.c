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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#endif 

#if defined(LDM_PCI)
#include <linux/pci.h>
#endif 

#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
#include <asm/uaccess.h>
#endif

#include "img_defs.h"
#include "services.h"
#include "kerneldisplay.h"
#include "kernelbuffer.h"
#include "syscommon.h"
#include "pvrmmap.h"
#include "mutils.h"
#include "mm.h"
#include "mmap.h"
#include "mutex.h"
#include "pvr_debug.h"
#include "srvkm.h"
#include "perproc.h"
#include "handle.h"
#include "pvr_bridge_km.h"
#include "proc.h"
#include "pvrmodule.h"
#include "private_data.h"

#if defined(SUPPORT_DRI_DRM)
#include "pvr_drm.h"
#endif
#define DRVNAME		"pvrsrvkm"
#define DEVNAME		"pvrsrvkm"


MODULE_SUPPORTED_DEVICE(DEVNAME);
#ifdef DEBUG
static IMG_INT debug = DBGPRIV_WARNING;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
#include <linux/moduleparam.h>
module_param(debug, int, 0);
#else
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Sets the level of debug output (default=0x4)");
#endif
#endif


IMG_VOID PVRDebugSetLevel(IMG_UINT32 uDebugLevel);

extern IMG_BOOL PVRGetDisplayClassJTable(PVRSRV_DC_DISP2SRV_KMJTABLE *psJTable);
extern IMG_BOOL PVRGetBufferClassJTable(PVRSRV_BC_BUFFER2SRV_KMJTABLE *psJTable);
EXPORT_SYMBOL(PVRGetDisplayClassJTable);
EXPORT_SYMBOL(PVRGetBufferClassJTable);


static IMG_INT AssignedMajorNumber;

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
static struct class *psPvrClass;
#endif

extern IMG_INT32 PVRSRV_BridgeDispatchKM(struct file *file, IMG_UINT cmd, IMG_UINT32 arg);
static IMG_INT PVRSRVOpen(struct inode* pInode, struct file* pFile);
static IMG_INT PVRSRVRelease(struct inode* pInode, struct file* pFile);

PVRSRV_LINUX_MUTEX gPVRSRVLock;

static struct file_operations pvrsrv_fops = {
	owner:THIS_MODULE,
	unlocked_ioctl:PVRSRV_BridgeDispatchKM,
	open:PVRSRVOpen,
	release:PVRSRVRelease,
	mmap:PVRMMap,
};


#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
static IMG_UINT32 gPVRPowerLevel;
#endif

#if defined(LDM_PLATFORM) || defined(LDM_PCI)

#if defined(LDM_PLATFORM)
#define	LDM_DEV	struct platform_device
#define	LDM_DRV	struct platform_driver
#if defined(LDM_PCI)
#undef	LDM_PCI
#endif 
#endif 

#if defined(LDM_PCI)
#define	LDM_DEV	struct pci_dev
#define	LDM_DRV	struct pci_driver
#endif 

#if defined(LDM_PLATFORM)
static IMG_INT PVRSRVDriverRemove(LDM_DEV *device);
static IMG_INT PVRSRVDriverProbe(LDM_DEV *device);
#endif
#if defined(LDM_PCI)
static IMG_VOID PVRSRVDriverRemove(LDM_DEV *device);
static IMG_INT PVRSRVDriverProbe(LDM_DEV *device, const struct pci_device_id *id);
#endif
static IMG_INT PVRSRVDriverSuspend(LDM_DEV *device, pm_message_t state);
static IMG_VOID PVRSRVDriverShutdown(LDM_DEV *device);
static IMG_INT PVRSRVDriverResume(LDM_DEV *device);

#if defined(LDM_PCI)
struct pci_device_id powervr_id_table[] __devinitdata = {
	{ PCI_DEVICE(SYS_SGX_DEV_VENDOR_ID, SYS_SGX_DEV_DEVICE_ID) },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, powervr_id_table);
#endif

static LDM_DRV powervr_driver = {
#if defined(LDM_PLATFORM)
	.driver = {
		.name		= DRVNAME,
	},
#endif
#if defined(LDM_PCI)
	.name		= DRVNAME,
	.id_table = powervr_id_table,
#endif
	.probe		= PVRSRVDriverProbe,
#if defined(LDM_PLATFORM)
	.remove		= PVRSRVDriverRemove,
#endif
#if defined(LDM_PCI)
	.remove		= __devexit_p(PVRSRVDriverRemove),
#endif
	.suspend	= PVRSRVDriverSuspend,
	.resume		= PVRSRVDriverResume,
	.shutdown	= PVRSRVDriverShutdown,
};

LDM_DEV *gpsPVRLDMDev;

#if defined(LDM_PLATFORM)
static IMG_VOID PVRSRVDeviceRelease(struct device *device);

static struct platform_device powervr_device = {
	.name			= DEVNAME,
	.id				= -1,
	.dev 			= {
		.release		= PVRSRVDeviceRelease
	}
};
#endif 

#if defined(LDM_PLATFORM)
static IMG_INT PVRSRVDriverProbe(LDM_DEV *pDevice)
#endif
#if defined(LDM_PCI)
static IMG_INT __devinit PVRSRVDriverProbe(LDM_DEV *pDevice, const struct pci_device_id *id)
#endif
{
	SYS_DATA *psSysData;

	PVR_TRACE(("PVRSRVDriverProbe(pDevice=%p)", pDevice));

#if 0
	
	if (PerDeviceSysInitialise((IMG_PVOID)pDevice) != PVRSRV_OK)
	{
		return -EINVAL;
	}
#endif	
	
	if (SysAcquireData(&psSysData) != PVRSRV_OK)
	{
		gpsPVRLDMDev = pDevice;

		if (SysInitialise() != PVRSRV_OK)
		{
			return -ENODEV;
		}
	}

	return 0;
}


#if defined (LDM_PLATFORM)
static IMG_INT PVRSRVDriverRemove(LDM_DEV *pDevice)
#endif
#if defined(LDM_PCI)
static IMG_VOID __devexit PVRSRVDriverRemove(LDM_DEV *pDevice)
#endif
{
	SYS_DATA *psSysData;

	PVR_TRACE(("PVRSRVDriverRemove(pDevice=%p)", pDevice));

	if (SysAcquireData(&psSysData) == PVRSRV_OK)
	{
#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
		if (gPVRPowerLevel != 0)
		{
			if (PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D0) == PVRSRV_OK)
			{
				gPVRPowerLevel = 0;
			}
		}
#endif
		SysDeinitialise(psSysData);

		gpsPVRLDMDev = IMG_NULL;
	}

#if 0
	if (PerDeviceSysDeInitialise((IMG_PVOID)pDevice) != PVRSRV_OK)
	{
		return -EINVAL;
	}
#endif

#if defined (LDM_PLATFORM)
	return 0;
#endif
#if defined (LDM_PCI)
	return;
#endif
}


static IMG_VOID PVRSRVDriverShutdown(LDM_DEV *pDevice)
{
	PVR_TRACE(("PVRSRVDriverShutdown(pDevice=%p)", pDevice));

	(IMG_VOID) PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D3);
}


static IMG_INT PVRSRVDriverSuspend(LDM_DEV *pDevice, pm_message_t state)
{
#if !(defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL))
	PVR_TRACE(( "PVRSRVDriverSuspend(pDevice=%p)", pDevice));

	if (PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D3) != PVRSRV_OK)
	{
		return -EINVAL;
	}
#endif
	return 0;
}


static IMG_INT PVRSRVDriverResume(LDM_DEV *pDevice)
{
#if !(defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL))
	PVR_TRACE(("PVRSRVDriverResume(pDevice=%p)", pDevice));

	if (PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D0) != PVRSRV_OK)
	{
		return -EINVAL;
	}
#endif
	return 0;
}


#if defined(LDM_PLATFORM)
static IMG_VOID PVRSRVDeviceRelease(struct device *pDevice)
{
	PVR_DPF((PVR_DBG_WARNING, "PVRSRVDeviceRelease(pDevice=%p)", pDevice));
}
#endif 
#endif 


#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
IMG_INT PVRProcSetPowerLevel(struct file *file, const IMG_CHAR *buffer, IMG_UINT32 count, IMG_VOID *data)
{
	IMG_CHAR data_buffer[2];
	IMG_UINT32 PVRPowerLevel;

	if (count != sizeof(data_buffer))
	{
		return -EINVAL;
	}
	else
	{
		if (copy_from_user(data_buffer, buffer, count))
			return -EINVAL;
		if (data_buffer[count - 1] != '\n')
			return -EINVAL;
		PVRPowerLevel = data_buffer[0] - '0';
		if (PVRPowerLevel != gPVRPowerLevel)
		{
			if (PVRPowerLevel != 0)
			{
				if (PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D3) != PVRSRV_OK)
				{
					return -EINVAL;
				}
			}
			else
			{
				if (PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D0) != PVRSRV_OK)
				{
					return -EINVAL;
				}
			}

			gPVRPowerLevel = PVRPowerLevel;
		}
	}
	return (count);
}

IMG_INT PVRProcGetPowerLevel(IMG_CHAR *page, IMG_CHAR **start, off_t off, IMG_INT count, IMG_INT *eof, IMG_VOID *data)
{
	if (off == 0) {
		*start = (IMG_CHAR *)1;
		return printAppend(page, count, 0, "%lu\n", gPVRPowerLevel);
	}
	*eof = 1;
	return 0;
}
#endif

static IMG_INT PVRSRVOpen(struct inode unref__ * pInode, struct file *pFile)
{
	PVRSRV_FILE_PRIVATE_DATA *psPrivateData;
	IMG_HANDLE hBlockAlloc;
	IMG_INT iRet = -ENOMEM;
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32PID;

	LinuxLockMutex(&gPVRSRVLock);

	ui32PID = OSGetCurrentProcessIDKM();

	if (PVRSRVProcessConnect(ui32PID) != PVRSRV_OK)
		goto err_unlock;

	eError = OSAllocMem(PVRSRV_OS_NON_PAGEABLE_HEAP,
						sizeof(PVRSRV_FILE_PRIVATE_DATA),
						(IMG_PVOID *)&psPrivateData,
						&hBlockAlloc);

	if(eError != PVRSRV_OK)
		goto err_unlock;

#if defined(PVR_SECURE_FD_EXPORT)
	psPrivateData->hKernelMemInfo = NULL;
#endif
	psPrivateData->ui32OpenPID = ui32PID;
	psPrivateData->hBlockAlloc = hBlockAlloc;
	pFile->private_data = psPrivateData;

	iRet = 0;
err_unlock:	
	LinuxUnLockMutex(&gPVRSRVLock);
	return iRet;
}


static IMG_INT PVRSRVRelease(struct inode unref__ * pInode, struct file *pFile)
{
	PVRSRV_FILE_PRIVATE_DATA *psPrivateData;

	LinuxLockMutex(&gPVRSRVLock);

	psPrivateData = pFile->private_data;

	
	PVRSRVProcessDisconnect(psPrivateData->ui32OpenPID);

	OSFreeMem(PVRSRV_OS_NON_PAGEABLE_HEAP,
			  sizeof(PVRSRV_FILE_PRIVATE_DATA),
			  psPrivateData, psPrivateData->hBlockAlloc);

	LinuxUnLockMutex(&gPVRSRVLock);
	return 0;
}


static IMG_INT __init PVRCore_Init(IMG_VOID)
{
	IMG_INT error;
#if !(defined(LDM_PLATFORM) || defined(LDM_PCI))
	PVRSRV_ERROR eError;
#else
	struct device *psDev;
#endif

	PVR_TRACE(("PVRCore_Init"));

	LinuxInitMutex(&gPVRSRVLock);

#ifdef DEBUG
	PVRDebugSetLevel(debug);
#endif

	if (CreateProcEntries ())
	{
		error = -ENOMEM;
		return error;
	}

	PVRLinuxMUtilsInit();

	if(LinuxMMInit() != PVRSRV_OK)
	{
		error = -ENOMEM;
		goto init_failed;
	}

	LinuxBridgeInit();

	PVRMMapInit();

#if defined(LDM_PLATFORM) || defined(LDM_PCI)

#if defined(LDM_PLATFORM)
	if ((error = platform_driver_register(&powervr_driver)) != 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRCore_Init: unable to register platform driver (%d)", error));

		goto init_failed;
	}

#ifdef NO
	if ((error = platform_device_register(&powervr_device)) != 0)
	{
		platform_driver_unregister(&powervr_driver);

		PVR_DPF((PVR_DBG_ERROR, "PVRCore_Init: unable to register platform device (%d)", error));

		goto init_failed;
	}
#endif
#endif 

#if defined(LDM_PCI)
	if ((error = pci_register_driver(&powervr_driver)) != 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRCore_Init: unable to register PCI driver (%d)", error));

		goto init_failed;
	}
#endif 

#else 
	
	if ((eError = SysInitialise()) != PVRSRV_OK)
	{
		error = -ENODEV;
#if defined(TCF_REV) && (TCF_REV == 110)
		if(eError == PVRSRV_ERROR_NOT_SUPPORTED)
		{
			printk("\nAtlas wrapper (FPGA image) version mismatch");
			error = -ENODEV;
		}
#endif
		goto init_failed;
	}
#endif 
#if defined(SUPPORT_DRI_DRM)
	if(PVRSRVDrmInit() != PVRSRV_OK)
	{
			error = -ENODEV;
			goto sys_deinit;
	}	
#endif

	AssignedMajorNumber = register_chrdev(0, DEVNAME, &pvrsrv_fops);

	if (AssignedMajorNumber <= 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRCore_Init: unable to get major number"));

		error = -EBUSY;
		goto drm_deinit;
	}

	PVR_TRACE(("PVRCore_Init: major device %d", AssignedMajorNumber));

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
	
	psPvrClass = class_create(THIS_MODULE, "pvr");

	if (IS_ERR(psPvrClass))
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRCore_Init: unable to create class (%ld)", PTR_ERR(psPvrClass)));
		error = -EBUSY;
		goto unregister_device;
	}

	psDev = device_create(psPvrClass, NULL, MKDEV(AssignedMajorNumber, 0),
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
				  NULL,
#endif 
				  DEVNAME);
	if (IS_ERR(psDev))
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRCore_Init: unable to create device (%ld)", PTR_ERR(psDev)));
		error = -EBUSY;
		goto destroy_class;
	}
#endif 

	return 0;

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
destroy_class:
	class_destroy(psPvrClass);
unregister_device:
#endif
	unregister_chrdev(AssignedMajorNumber, DRVNAME);
drm_deinit:
#if defined(SUPPORT_DRI_DRM)
	PVRSRVDrmExit();
sys_deinit:
#endif
#if defined(LDM_PLATFORM) || defined(LDM_PCI)
#if defined(LDM_PCI)
	pci_unregister_driver(&powervr_driver);
#endif

#if defined (LDM_PLATFORM)
	platform_device_unregister(&powervr_device);
	platform_driver_unregister(&powervr_driver);
#endif

#else	
	
	{
		SYS_DATA *psSysData;

		SysAcquireData(&psSysData);
		if (psSysData != IMG_NULL)
		{
			SysDeinitialise(psSysData);
		}
	}
#endif	
init_failed:
	PVRMMapCleanup();
	LinuxMMCleanup();
	LinuxBridgeDeInit();
	RemoveProcEntries();

	return error;

} 


static IMG_VOID __exit PVRCore_Cleanup(IMG_VOID)
{
	SYS_DATA *psSysData;

	PVR_TRACE(("PVRCore_Cleanup"));

	SysAcquireData(&psSysData);

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
	device_destroy(psPvrClass, MKDEV(AssignedMajorNumber, 0));
	class_destroy(psPvrClass);
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,22))
	if (
#endif	
		unregister_chrdev(AssignedMajorNumber, DRVNAME)
#if !(LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,22))
								;
#else	
								)
	{
		PVR_DPF((PVR_DBG_ERROR," can't unregister device major %d", AssignedMajorNumber));
	}
#endif	

#if defined(LDM_PLATFORM) || defined(LDM_PCI)

#if defined(LDM_PCI)
	pci_unregister_driver(&powervr_driver);
#endif

#if defined (LDM_PLATFORM)
	platform_device_unregister(&powervr_device);
	platform_driver_unregister(&powervr_driver);
#endif

#else 
#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
	if (gPVRPowerLevel != 0)
	{
		if (PVRSRVSetPowerStateKM(PVRSRV_POWER_STATE_D0) == PVRSRV_OK)
		{
			gPVRPowerLevel = 0;
		}
	}
#endif
	
	SysDeinitialise(psSysData);
#endif 

	PVRMMapCleanup();

	LinuxMMCleanup();

	LinuxBridgeDeInit();

	RemoveProcEntries();

	PVR_TRACE(("PVRCore_Cleanup: unloading"));
#if defined(SUPPORT_DRI_DRM)
	PVRSRVDrmExit();
#endif
}

module_init(PVRCore_Init);
module_exit(PVRCore_Cleanup);

