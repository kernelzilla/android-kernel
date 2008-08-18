/*
 * linux/drivers/dsp/bridge/rmgr/drv_interface.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== linux_driver.c ========
 *  Description:
 *      DSP/BIOS Bridge driver interface.
 *
 *  Public Functions:
 *      driver_init
 *      driver_exit
 *      driver_open
 *      driver_release
 *      driver_ioctl
 *      driver_mmap
 *
 *! Revision History
 *! ================
 *! 21-Apr-2004 map   Deprecated use of MODULE_PARM for kernel versions
 *!		   greater than 2.5, use module_param.
 *! 08-Mar-2004 sb    Added the dsp_debug argument, which keeps the DSP in self
 *!		   loop after image load and waits in a loop for DSP to start
 *! 16-Feb-2004 vp    Deprecated the usage of MOD_INC_USE_COUNT and
 *! 						MOD_DEC_USE_COUNT
 *!		   for kernel versions greater than 2.5
 *! 20-May-2003 vp    Added unregister functions for the DPM.
 *! 24-Mar-2003 sb    Pass pid instead of driverContext to DSP_Close
 *! 24-Mar-2003 vp    Added Power Management support.
 *! 21-Mar-2003 sb    Configure SHM size using insmod argument shm_size
 *! 10-Feb-2003 vp    Updated based on code review comments
 *! 18-Oct-2002 sb    Created initial version
 */

/*  ----------------------------------- Host OS */

#include <host_os.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#ifdef MODULE
#include <linux/module.h>
#endif

#include <linux/device.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
#include <asm/arch/resource.h>
#include <asm/arch/prcm_34xx.h>
#endif
#endif

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <gt.h>
#include <dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <services.h>
#include <sync.h>
#include <reg.h>
#include <csl.h>
#include <prcs.h>

/*  ----------------------------------- Platform Manager */
#include <wcdioctl.h>
#include <_dcd.h>
#include <dspdrv.h>
#include <dbreg.h>

/*  ----------------------------------- Resource Manager */
#include <pwr.h>

/*  ----------------------------------- This */
#include <drv_interface.h>

#ifndef RES_CLEANUP_DISABLE
#include <cfg.h>
#include <resourcecleanup.h>
#include <chnl.h>
#include <proc.h>
#include <cfg.h>
#include <dev.h>
#include <drvdefs.h>
#include <drv.h>
#include <dbreg.h>
#endif

#define BRIDGE_NAME "C6410"
/*  ----------------------------------- Globals */
#define DRIVER_NAME  "DspBridge"
#define DRIVER_MAJOR 0		/* Linux assigns our Major device number */
#define DRIVER_MINOR 0		/* Linux assigns our Major device number */
s32 dsp_debug;

/* This is a test variable used by Bridge to test different sleep states */
static s32 dsp_test_sleepstate;
struct bridge_dev {
	struct cdev cdev;
};

static struct bridge_dev *bridge_device;

static struct class *bridge_class;

static u32 driverContext;
static char *GT_str;
static s32 driver_major = DRIVER_MAJOR;
static s32 driver_minor = DRIVER_MINOR;
static char *base_img;
char *iva_img;
static char *num_procs = "C55=1";
static s32 shm_size = 0x400000;	/* 4 MB */
static u32 phys_mempool_base = 0x87000000;
static u32 phys_mempool_size = 0x600000;

#if !defined(CONFIG_ARCH_OMAP2430) && !defined(CONFIG_ARCH_OMAP3430)
static int tc_wordswapon = 1;	/* Default value is always TRUE */
#else
static int tc_wordswapon;	/* Default value is always true */
#endif



#ifndef CONFIG_DISABLE_BRIDGE_PM
struct omap34xx_bridge_suspend_data {
	int suspended;
	wait_queue_head_t suspend_wq;
};

static struct omap34xx_bridge_suspend_data bridge_suspend_data;

int omap34xxbridge_suspend_lockout(struct omap34xx_bridge_suspend_data *s,
				  struct file *f)
{
	if ((s)->suspended) {
		if ((f)->f_flags & O_NONBLOCK)
			return DSP_EDPMSUSPEND;
		wait_event_interruptible((s)->suspend_wq, (s)->suspended == 0);
	}
	return 0;
}

#endif

#ifdef DEBUG
module_param(GT_str, charp, 0);
MODULE_PARM_DESC(GT_str, "GT string, default = NULL");

module_param(dsp_debug, int, 0);
MODULE_PARM_DESC(dsp_debug, "Wait after loading DSP image. default = false");
#endif

module_param(driver_major, int, 0);	/* Driver's major number */
MODULE_PARM_DESC(driver_major, "Major device number, default = 0 (auto)");

module_param(driver_minor, int, 0);	/* Driver's major number */
MODULE_PARM_DESC(driver_minor, "Minor device number, default = 0 (auto)");

module_param(dsp_test_sleepstate, int, 0);
MODULE_PARM_DESC(dsp_test_sleepstate, "DSP Sleep state = 0");

module_param(base_img, charp, 0);
MODULE_PARM_DESC(base_img, "DSP base image, default = NULL");

module_param(shm_size, int, 0);
MODULE_PARM_DESC(shm_size, "SHM size, default = 4 MB, minimum = 64 KB");

module_param(phys_mempool_base, uint, 0);
MODULE_PARM_DESC(phys_mempool_base,
		"Physical memory pool base passed to driver");

module_param(phys_mempool_size, uint, 0);
MODULE_PARM_DESC(phys_mempool_size,
		"Physical memory pool size passed to driver");
module_param(tc_wordswapon, int, 0);
MODULE_PARM_DESC(tc_wordswapon, "TC Word Swap Option. default = 0");

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

static char *driver_name = DRIVER_NAME;
static struct GT_Mask driverTrace;

static struct file_operations bridge_fops = {
	.open		= bridge_open,
	.release	= bridge_release,
	.ioctl		= bridge_ioctl,
	.mmap		= bridge_mmap,
};

#ifndef CONFIG_DISABLE_BRIDGE_PM
u32 timeOut = 1000;

static int bridge_suspend(struct platform_device *pdev, pm_message_t state);
static int bridge_resume(struct platform_device *pdev);
#endif

static void bridge_free(struct device *dev);

static int omap34xx_bridge_probe(struct platform_device *dev);

static int omap34xx_bridge_probe(struct platform_device *dev)
{
	return 0;
}

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
static struct platform_device omap_dspbridge_dev = {
		.name = BRIDGE_NAME,
		.id = -1,
		.num_resources = 0,
		.dev = {
		.release = bridge_free,
		},
		.resource = NULL,
};


#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
/* The number of OPPs supported in the system */
s32 dsp_max_opps = CO_VDD1_OPP5-2;
u32 vdd1_dsp_freq[6][4] = {

	 {0, 0, 0, 0},

	 {0, 90000, 0, 86000},

	{0, 180000, 80000, 170000},

	{0, 360000, 160000, 340000},

	{0, 396000, 325000, 376000},

	{0, 430000, 355000, 430000},
};

/* The handle for setting constraints */
struct constraint_handle *dsp_constraint_handle;
struct constraint_handle *mpu_constraint_handle;

static int dspbridge_post_scale(struct notifier_block *op, unsigned long level,
				void *ptr)
{
#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
	PWR_PM_PostScale(PRCM_VDD1, level);
#endif
#endif
	return 0;
}


static struct notifier_block omap34xxbridge_post_scale = {
	.notifier_call = dspbridge_post_scale,
	NULL,
};
static struct constraint_id cnstr_id_vdd1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};
#endif
#endif

static struct platform_driver bridge_driver_ldm = {
      .driver = {
	      .owner	= THIS_MODULE,
	      .name     = BRIDGE_NAME,
	 },
      .probe = omap34xx_bridge_probe,
#ifndef CONFIG_DISABLE_BRIDGE_PM
      .suspend = bridge_suspend,
      .resume = bridge_resume,
#endif
      .shutdown = NULL,
      .remove = NULL,

};

#endif

/* Initialization routine. Executed when the driver is loaded (as a kernel
 * module), or when the system is booted (when included as part of the kernel
 * image). */
static int __init bridge_init(void)
{
	int status;
	u32 initStatus;
	u32 temp;
	dev_t   dev = 0 ;
	int     result;

	/* use 2.6 device model */
	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		result = register_chrdev_region(dev, 1, driver_name);
	} else {
		result = alloc_chrdev_region(&dev, driver_minor, 1,
					    driver_name);
		driver_major = MAJOR(dev);
	}

	if (result < 0) {
		GT_1trace(driverTrace, GT_7CLASS, "bridge_init: "
				"Can't get Major %d \n", driver_major);
		return result;
	}

	bridge_device = kmalloc(sizeof(struct bridge_dev), GFP_KERNEL);
	if (!bridge_device) {
		result = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		return result;
	}
	memset(bridge_device, 0, sizeof(struct bridge_dev));
	cdev_init(&bridge_device->cdev, &bridge_fops);
	bridge_device->cdev.owner = THIS_MODULE;
	bridge_device->cdev.ops = &bridge_fops;

	status = cdev_add(&bridge_device->cdev, dev, 1);

	if (status) {
		GT_0trace(driverTrace, GT_7CLASS,
				"Failed to add the bridge device \n");
		return status;
	}

	/* udev support */
	bridge_class = class_create(THIS_MODULE, "ti_bridge");

	if (IS_ERR(bridge_class))
		GT_0trace(driverTrace, GT_7CLASS,
				"Error creating bridge class \n");

	device_create(bridge_class, NULL, MKDEV(driver_major,
			driver_minor), NULL, "DspBridge");

	GT_init();
	GT_create(&driverTrace, "LD");

#ifdef DEBUG
	if (GT_str)
		GT_set(GT_str);

#else
#if (defined DDSP_DEBUG_PRODUCT) && GT_TRACE
	GT_set("**=67");
#endif
#endif


	GT_0trace(driverTrace, GT_ENTER, "-> driver_init\n");
	status = platform_driver_register(&bridge_driver_ldm);
	if (!status)
		status = platform_device_register(&omap_dspbridge_dev);

#ifndef CONFIG_DISABLE_BRIDGE_PM
	/* Initialize the wait queue */
	if (!status) {
		bridge_suspend_data.suspended = 0;
		init_waitqueue_head(&bridge_suspend_data.suspend_wq);
	}
#endif

	SERVICES_Init();

	/*  Autostart flag.  This should be set to true if the DSP image should
	 *  be loaded and run during bridge module initialization  */

	if (base_img) {
		temp = true;
		REG_SetValue(NULL, NULL, AUTOSTART, REG_DWORD, (u8 *)&temp,
			    sizeof(temp));
		REG_SetValue(NULL, NULL, DEFEXEC, REG_SZ, (u8 *)base_img,
			    CSL_Strlen(base_img) + 1);
	} else {
		temp = false;
		REG_SetValue(NULL, NULL, AUTOSTART, REG_DWORD, (u8 *)&temp,
			    sizeof(temp));
		REG_SetValue(NULL, NULL, DEFEXEC, REG_SZ, (u8 *) "\0", (u32)2);
	}
	REG_SetValue(NULL, NULL, NUMPROCS, REG_SZ, (u8 *) num_procs,
		    CSL_Strlen(num_procs) + 1);

	if (shm_size >= 0x10000) {	/* 64 KB */
		initStatus = REG_SetValue(NULL, NULL, SHMSIZE, REG_DWORD,
					  (u8 *)&shm_size, sizeof(shm_size));
	} else {
		initStatus = DSP_EINVALIDARG;
		status = -1;
		GT_0trace(driverTrace, GT_7CLASS,
			 "SHM size must be atleast 64 KB\n");
	}
	GT_1trace(driverTrace, GT_7CLASS,
		 "requested shm_size = 0x%x\n", shm_size);

	if (phys_mempool_base > 0x0) {
		initStatus = REG_SetValue(NULL, NULL, PHYSMEMPOOLBASE,
					 REG_DWORD, (u8 *)&phys_mempool_base,
					 sizeof(phys_mempool_base));
	}
	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_base = 0x%x \n",
		 phys_mempool_base);

	if (phys_mempool_size > 0x0) {
		initStatus = REG_SetValue(NULL, NULL, PHYSMEMPOOLSIZE,
					 REG_DWORD, (u8 *)&phys_mempool_size,
					 sizeof(phys_mempool_size));
	}
	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_size = 0x%x \n",
		 phys_mempool_base);
	if ((phys_mempool_base > 0x0) && (phys_mempool_size > 0x0))
		MEM_ExtPhysPoolInit(phys_mempool_base, phys_mempool_size);
	if (tc_wordswapon) {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is enabled\n");
		REG_SetValue(NULL, NULL, TCWORDSWAP, REG_DWORD,
			    (u8 *)&tc_wordswapon, sizeof(tc_wordswapon));
	} else {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is disabled\n");
		REG_SetValue(NULL, NULL, TCWORDSWAP,
			    REG_DWORD, (u8 *)&tc_wordswapon,
			    sizeof(tc_wordswapon));
	}
	if (DSP_SUCCEEDED(initStatus)) {
		driverContext = DSP_Init(&initStatus);
		if (DSP_FAILED(initStatus)) {
			status = -1;
			GT_0trace(driverTrace, GT_7CLASS,
				 "DSP/BIOS Bridge initialization Failed\n");
		} else {
			GT_0trace(driverTrace, GT_5CLASS,
					"DSP/BIOS Bridge driver loaded\n");
		}
 #ifndef CONFIG_DISABLE_BRIDGE_PM
 #ifndef CONFIG_DISABLE_BRIDGE_DVFS
		/* Register for the constraints */
		dsp_constraint_handle = constraint_get("dspbridge",
						      &cnstr_id_vdd1);
		constraint_register_post_notification(dsp_constraint_handle,
						 &omap34xxbridge_post_scale,
						 CO_VDD1_OPP5 + 1);
		mpu_constraint_handle = constraint_get("mpubridge",
						      &cnstr_id_vdd1);
		constraint_register_post_notification(mpu_constraint_handle,
						 &omap34xxbridge_post_scale,
						 CO_VDD1_OPP5 + 1);
#endif
#endif
	}

	DBC_Assert(status == 0);
	DBC_Assert(DSP_SUCCEEDED(initStatus));
	GT_0trace(driverTrace, GT_ENTER, " <- driver_init\n");
	return status;
}

/*  This function is invoked during unlinking of the bridge module from the
 *  kernel. Bridge resources are freed in this function. */
static void __exit bridge_exit(void)
{
	dev_t devno;
	bool ret;
	GT_0trace(driverTrace, GT_ENTER, "-> driver_exit\n");

#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
	/* remove the constraints */
	if (dsp_constraint_handle != NULL) {
		GT_0trace(driverTrace, GT_7CLASS,
			 "bridge_exit: remove constraints\n");
		constraint_remove(dsp_constraint_handle);
		constraint_unregister_post_notification(dsp_constraint_handle,
						&omap34xxbridge_post_scale,
						CO_VDD1_OPP5 + 1);
		constraint_put(dsp_constraint_handle);
		dsp_constraint_handle = NULL;
	} else {
		GT_0trace(driverTrace, GT_7CLASS,
			 "dsp_constraint_handle is NULL\n");

	}
	if (mpu_constraint_handle != NULL) {
		GT_0trace(driverTrace, GT_7CLASS,
			 "bridge_exit: remove constraints\n");
		constraint_remove(mpu_constraint_handle);
		constraint_unregister_post_notification(mpu_constraint_handle,
						&omap34xxbridge_post_scale,
						CO_VDD1_OPP5 + 1);
		constraint_put(mpu_constraint_handle);
		mpu_constraint_handle = NULL;
	} else {
		GT_0trace(driverTrace, GT_7CLASS,
			 "mpu_constraint_handle is NULL\n");

	}
#endif /*#ifndef CONFIG_DISABLE_BRIDGE_DVFS*/
#endif /*#ifndef CONFIG_DISABLE_BRIDGE_PM*/
	/* unregister bridge driver */
	platform_device_unregister(&omap_dspbridge_dev);
	platform_driver_unregister(&bridge_driver_ldm);

	if (driverContext) {
		ret = DSP_Deinit(driverContext);
		driverContext = 0;

		DBC_Assert(ret == true);
	}
	SERVICES_Exit();
	GT_exit();

	devno = MKDEV(driver_major, driver_minor);
	if (bridge_device) {
		cdev_del(&bridge_device->cdev);
		kfree(bridge_device);
	}
	unregister_chrdev_region(devno, 1);
	if (bridge_class) {
		/* remove the device from sysfs */
		device_destroy(bridge_class, MKDEV(driver_major, driver_minor));
		class_destroy(bridge_class);

	}
}

/* This function is called when an application opens handle to the
 * bridge driver. */

static int bridge_open(struct inode *ip, struct file *filp)
{
	int status = 0;
#ifndef RES_CLEANUP_DISABLE
	HANDLE	     hProcess;
	DSP_STATUS dsp_status = DSP_SOK;
	HANDLE	     hDrvObject = NULL;
	struct PROCESS_CONTEXT    *pPctxt = NULL;
	struct PROCESS_CONTEXT	*pTmp = NULL;
	struct PROCESS_CONTEXT    *pCtxtclosed = NULL;
	struct PROCESS_CONTEXT    *pCtxttraverse = NULL;
	struct task_struct *tsk = NULL;
	GT_0trace(driverTrace, GT_ENTER, "-> driver_open\n");
	dsp_status = CFG_GetObject((u32 *)&hDrvObject, REG_DRV_OBJECT);

	/* Checking weather task structure for all process existing
	 * in the process context list If not removing those processes*/
	if (DSP_FAILED(dsp_status))
		goto func_cont;

	DRV_GetProcCtxtList(&pCtxtclosed, (struct DRV_OBJECT *)hDrvObject);
	while (pCtxtclosed != NULL) {
		tsk = find_task_by_vpid(pCtxtclosed->pid);

		if ((tsk == NULL) || (tsk->exit_state == EXIT_ZOMBIE)) {

			GT_1trace(driverTrace, GT_5CLASS,
				 "***Task structure not existing for "
				 "process***%d\n", pCtxtclosed->pid);
			DRV_RemoveAllResources(pCtxtclosed);
			if (pCtxtclosed->hProcessor != NULL) {
				DRV_GetProcCtxtList(&pCtxttraverse,
					    (struct DRV_OBJECT *)hDrvObject);
				if (pCtxttraverse->next == NULL) {
					PROC_Detach(pCtxtclosed->hProcessor);
				} else {
					if ((pCtxtclosed->pid ==
					  pCtxttraverse->pid) &&
					  (pCtxttraverse->next != NULL)) {
						pCtxttraverse =
							pCtxttraverse->next;
					}
					while ((pCtxttraverse != NULL) &&
					     (pCtxtclosed->hProcessor
					     != pCtxttraverse->hProcessor)) {
						pCtxttraverse =
							pCtxttraverse->next;
						if ((pCtxttraverse != NULL) &&
						  (pCtxtclosed->pid ==
						  pCtxttraverse->pid)) {
							pCtxttraverse =
							   pCtxttraverse->next;
						}
					}
					if (pCtxttraverse == NULL) {
						PROC_Detach
						     (pCtxtclosed->hProcessor);
					}
				}
			}
			pTmp = pCtxtclosed->next;
			DRV_RemoveProcContext((struct DRV_OBJECT *)hDrvObject,
					     pCtxtclosed,
					     (void *)pCtxtclosed->pid);
		} else {
			pTmp = pCtxtclosed->next;
		}
		pCtxtclosed = pTmp;
	}
func_cont:
	dsp_status = CFG_GetObject((u32 *)&hDrvObject, REG_DRV_OBJECT);
	if (DSP_SUCCEEDED(dsp_status))
		dsp_status = DRV_InsertProcContext(
				(struct DRV_OBJECT *)hDrvObject, &pPctxt);

	if (pPctxt != NULL) {
		PRCS_GetCurrentHandle(&hProcess);
		DRV_ProcUpdatestate(pPctxt, PROC_RES_ALLOCATED);
		DRV_ProcSetPID(pPctxt, (s32) hProcess);
	}
#endif

	GT_0trace(driverTrace, GT_ENTER, " <- driver_open\n");
	return status;
}

/* This function is called when an application closes handle to the bridge
 * driver. */
static int bridge_release(struct inode *ip, struct file *filp)
{
	int status;
	HANDLE pid;

	GT_0trace(driverTrace, GT_ENTER, "-> driver_release\n");

	status = PRCS_GetCurrentHandle(&pid);

	if (DSP_SUCCEEDED(status))
		status = DSP_Close((u32) pid);


	(status == true) ? (status = 0) : (status = -1);

	GT_0trace(driverTrace, GT_ENTER, " <- driver_release\n");

	return status;
}

static void bridge_free(struct device *dev)
{
	/* nothing to Free */
}


/* This function provides IO interface to the bridge driver. */
static int bridge_ioctl(struct inode *ip, struct file *filp, unsigned int code,
		unsigned long args)
{
	int status;
	u32 retval = DSP_SOK;
	union Trapped_Args pBufIn;

	DBC_Require(filp != NULL);
#ifndef CONFIG_DISABLE_BRIDGE_PM
	status = omap34xxbridge_suspend_lockout(&bridge_suspend_data, filp);
	if (status != 0)
		return status;

#endif

	GT_0trace(driverTrace, GT_ENTER, " -> driver_ioctl\n");

	/* Deduct one for the CMD_BASE. */
	code = (code - 1);

	status = copy_from_user(&pBufIn, (union Trapped_Args *)args,
				sizeof(union Trapped_Args));

	if (status >= 0) {
		status = WCD_CallDevIOCtl(code, &pBufIn, &retval);

		if (DSP_SUCCEEDED(status)) {
			status = retval;
		} else {
			GT_1trace(driverTrace, GT_7CLASS,
				 "IOCTL Failed, code : 0x%x\n", code);
			status = -1;
		}

	}

	GT_0trace(driverTrace, GT_ENTER, " <- driver_ioctl\n");

	return status;
}

/* This function maps kernel space memory to user space memory. */
static int bridge_mmap(struct file *filp, struct vm_area_struct *vma)
{
#if GT_TRACE
	u32 offset = vma->vm_pgoff << PAGE_SHIFT;
#endif
	u32 status;

	DBC_Assert(vma->vm_start < vma->vm_end);

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	GT_6trace(driverTrace, GT_3CLASS,
		 "vm filp %p offset %lx start %lx end %lx"
		 " page_prot %lx flags %lx\n", filp, offset, vma->vm_start,
		 vma->vm_end, vma->vm_page_prot, vma->vm_flags);

	status = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (status != 0)
		status = -EAGAIN;

	return status;
}

#ifndef RES_CLEANUP_DISABLE
/* To remove all process resources before removing the process from the
 * process context list*/
DSP_STATUS DRV_RemoveAllResources(HANDLE hPCtxt)
{
	DSP_STATUS status = DSP_SOK;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	if (pCtxt != NULL) {
		DRV_RemoveAllSTRMResElements(pCtxt);
		DRV_RemoveAllNodeResElements(pCtxt);
		DRV_RemoveAllDMMResElements(pCtxt);
		DRV_ProcUpdatestate(pCtxt, PROC_RES_FREED);
	}
	return status;
}
#endif

#ifndef CONFIG_DISABLE_BRIDGE_PM

static int bridge_suspend(struct platform_device *pdev, pm_message_t state)
{
	u32 status = DSP_EFAIL;
	u32 command = PWR_EMERGENCYDEEPSLEEP;

	status = PWR_SleepDSP(command, timeOut);
	if (DSP_SUCCEEDED(status)) {
		bridge_suspend_data.suspended = 1;
		return 0;
	} else {
		return -1;
	}
}

static int bridge_resume(struct platform_device *pdev)
{
	u32 status = DSP_EFAIL;

	status = PWR_WakeDSP(timeOut);

	if (DSP_SUCCEEDED(status)) {
		bridge_suspend_data.suspended = 0;
		wake_up(&bridge_suspend_data.suspend_wq);

		return 0;
	} else {
		return -1;
	}
}

#endif
/* Bridge driver initialization and de-initialization functions */
module_init(bridge_init);
module_exit(bridge_exit);

