/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/**********************************************************************
 * panic.c
 *
 * A driver for forcing a panic on either the modem or Linux side of
 * an Android phone.
 *
 **********************************************************************/
/*===========================================================================

                        EDIT HISTORY FOR MODULE

  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.


when          who         what, where, why
--------     ------       ----------------------------------------------------------
2008-12-19   R Stoddard   Initial authoring of meta-procedure.


===========================================================================*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include "../../arch/arm/mach-msm/proc_comm.h"

MODULE_LICENSE("GPL");

dev_t forcepanic_dev;


#define PROCCOMM_MODEM_FORCEPANIC 1
#define PROCCOMM_MODEM_WDOGSTATUS 2


static int forcepanic_atoi(const char *s)
{
        int k = 0;

        k = 0;
        while (*s != '\0' && *s >= '0' && *s <= '9')
	{
                k = 10 * k + (*s - '0');
                s++;
        }
        return k;
}


ssize_t forcepanic_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{
        int y = forcepanic_atoi(buff);

	if(y >= 0)
	{
		if(y)	printk(KERN_EMERG "FORCEPANIC DRIVER: Forcing modem to panic via meta-mproc command.  %d ms pause. (%s)\n", y, buff);
		else	printk(KERN_EMERG "FORCEPANIC DRIVER: Forcing modem to panic via meta-mproc command immediately (%s).\n", buff);
		meta_proc(PROCCOMM_MODEM_FORCEPANIC, &y);
	        printk(KERN_EMERG "FORCEPANIC DRIVER: Done forcing modem to panic via meta-mproc command.\n");
	}
	else
	{
	        printk(KERN_INFO "FORCEPANIC DRIVER: Checking WDOG_STATUS variable.\n");
		meta_proc(PROCCOMM_MODEM_WDOGSTATUS, &y);
		printk(KERN_INFO "FORCEPANIC DRIVER: Got %x for WDOG_STATUS.\n", y);
	}
        return count;
}

#define LONGSTRING "This is a long string that should cause a kernel panic when it is copied to the null pointer that is given above.  If it doesn't then I'll be stumped as to how to crash this thing."


ssize_t forcepanic_read(struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{
	int x;
	char *nullptr = NULL;

	printk(KERN_EMERG "FORCEPANIC DRIVER: Forcing a panic on the Linux kernel.\n");
	for(x = 0; nullptr[x] = LONGSTRING[x]; x++);

	panic("***\tForcepanic driver forced a panic...   The string copy above didn't panic this?");

	printk(KERN_EMERG "FORCEPANIC DRIVER: Survived a panic??\n");
	return 0;
}



struct file_operations forcepanic_ops = {
.owner = THIS_MODULE,
.write = forcepanic_write,
.read  = forcepanic_read
};

struct cdev *mycdev;

static int forcepanic_init(void)
{
	int devno;
	int result = alloc_chrdev_region(&forcepanic_dev, 0, 1, "forcepanic");

	if(result)  return result;

	mycdev = cdev_alloc();
	cdev_init(mycdev, &forcepanic_ops);
	mycdev->owner = THIS_MODULE;
	mycdev->ops = &forcepanic_ops;
	devno = MKDEV(MAJOR(forcepanic_dev), MINOR(forcepanic_dev));
	result = cdev_add(mycdev, devno, 1);
	return 0;
}

static void forcepanic_exit(void)
{
	unregister_chrdev( MAJOR(forcepanic_dev),"forcepanic");
	unregister_chrdev_region(forcepanic_dev, 1);
}


module_init(forcepanic_init);
module_exit(forcepanic_exit);
