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
 * bppsensor.c
 *
 * A sensor for detecting BP panics while the kernel is not calling
 * proc_comm calls... like when in a voice call.
 *
 **********************************************************************/
/*===========================================================================

                        EDIT HISTORY FOR MODULE

  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.


when          who         what, where, why
--------     ------       ----------------------------------------------------------
2010-1-15   R Stoddard   Initial authoring.


===========================================================================*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

MODULE_LICENSE("GPL");



static int incall = 0;


static dev_t bppsensor_dev;
static struct cdev *bppsensor_cdev;


extern int (*msm_check_for_modem_crash)(void);




static void poll_bppanic(struct work_struct* imbecile);



DECLARE_DELAYED_WORK(bppsensor_work, poll_bppanic);

struct workqueue_struct *worker;


static void poll_bppanic(struct work_struct* dummy)
{
//	printk(KERN_INFO "bppsensor trying to sense a panic....\n");

	msm_check_for_modem_crash();
	if(incall) queue_delayed_work(worker, &bppsensor_work, HZ/10);
}



static int bppsensor_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "bppsensor has been opened.\n");
	if (msm_check_for_modem_crash)
	{
		incall = 1;
		queue_delayed_work(worker, &bppsensor_work, 2);
	}
	return 0;
}


static int bppsensor_close(struct inode *inode, struct file *file)
{
	if(incall == 0) return 0;
	printk(KERN_INFO "bppsensor has been closed.\n");
	incall = 0;   // dont care that an extra poll may happen.

	return 0;
}



struct file_operations bppsensor_ops = {
.owner   = THIS_MODULE,
.open    = bppsensor_open,
.release = bppsensor_close
};


static int bppsensor_init(void)
{
	int devno;
	int result = alloc_chrdev_region(&bppsensor_dev, 0, 1, "bppsensor");

	if(result)  return result;

	bppsensor_cdev = cdev_alloc();
	cdev_init(bppsensor_cdev, &bppsensor_ops);
	bppsensor_cdev->owner = THIS_MODULE;
	bppsensor_cdev->ops = &bppsensor_ops;
	devno = MKDEV(MAJOR(bppsensor_dev), MINOR(bppsensor_dev));
	result = cdev_add(bppsensor_cdev, devno, 1);

	worker = create_singlethread_workqueue("bppsensor_workthread");

	return 0;
}

static void bppsensor_exit(void)
{
	if(worker) destroy_workqueue(worker);
	unregister_chrdev( MAJOR(bppsensor_dev),"bppsensor");
	unregister_chrdev_region(bppsensor_dev, 1);
}


module_init(bppsensor_init);
module_exit(bppsensor_exit);
