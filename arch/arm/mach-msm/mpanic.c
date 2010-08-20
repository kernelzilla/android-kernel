/* arch/arm/mach-msm/mpanic.c
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mtd/mtd.h>
#include <linux/notifier.h>
#include <linux/mtd/mtd.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/console.h>
#include <linux/preempt.h>
#include <linux/rtc.h>
#include <linux/console.h>
#include <mot/mot_handover.h>
#include <linux/io.h>
#include "proc_comm.h"

#define MODEM_PANIC_RAW_SIZE (8*1024)
/* ASCII data will require 1byte/nibble */
#define MODEM_PANIC_SIZE (MODEM_PANIC_RAW_SIZE*2)
char *bp_panic_buffer;

struct mpanic_data {
	struct proc_dir_entry *last_kmsg;
};

static struct mpanic_data drv_ctx;
static struct work_struct proc_removal_work;
static DEFINE_MUTEX(drv_mutex);

static char bp_panic_header[] = "Modem Panic Data START\n";
static int bp_panic_size = MODEM_PANIC_SIZE + sizeof(bp_panic_header) - 1;

static int mpanic_proc_last_kmsg_read(char *buffer, char **start,
				      off_t offset, int count, int *peof,
				      void *dat)
{
	char *str = NULL;

	if (!count)
		return 0;

	if (!(unsigned char *) mot_handover_get_panic()) {
		printk(KERN_INFO "No Modem panic data detected!\n");
		return 0;
	} else {
		mutex_lock(&drv_mutex);

		/* We only support reading a max of Modem Panic Data size */
		if (count > bp_panic_size)
			count = bp_panic_size;
		if ((offset + count) > bp_panic_size) {
			count = bp_panic_size - offset;
			if (count <= 0) {
				printk(KERN_ERR "mpanic_proc_last_kmsg_read: count out-of-bounds!\n");
				mutex_unlock(&drv_mutex);
				return 0;
			}
		}

		if (!offset) {
			printk(KERN_INFO "Modem Panic detected!\n");
			sprintf(buffer, "%s", bp_panic_header);
			count = strlen(bp_panic_header);
		} else {
			if (!bp_panic_buffer) {
				printk(KERN_ERR "mpanic_proc_last_kmsg_read: bp_panic_buffer not allocated!\n");
				mutex_unlock(&drv_mutex);
				return 0;
			}
			/* account for header */
			str = bp_panic_buffer + offset
			    - strlen(bp_panic_header);
			if (str < bp_panic_buffer) {
				printk(KERN_ERR "mpanic_proc_last_kmsg_read: offset error!\n");
				mutex_unlock(&drv_mutex);
				return 0;
			}
			/* raw panic data */
			memcpy(buffer, str, count);

			*start = (char *) count;
		}
		if ((offset + count) == bp_panic_size)
			*peof = 1;

		mutex_unlock(&drv_mutex);
	}

	return count;
}


static void mpanic_remove_proc_work(struct work_struct *work)
{
	struct mpanic_data *ctx = &drv_ctx;

	mutex_lock(&drv_mutex);

	if (ctx->last_kmsg) {
		remove_proc_entry("last_kmsg", NULL);
		ctx->last_kmsg = NULL;
	}

	kfree(bp_panic_buffer);
	bp_panic_buffer = NULL;

	mutex_unlock(&drv_mutex);
}

static int mpanic_proc_write(struct file *file, const char __user * buffer,
			     unsigned long count, void *data)
{
	schedule_work(&proc_removal_work);
	return count;
}

static void mpanic_bp_panicdata_dmesg(void)
{
	int i;
	unsigned char __iomem *bp_panic_ptr = NULL;
	if ((unsigned char *) mot_handover_get_panic()) {
		bp_panic_buffer = kmalloc(MODEM_PANIC_SIZE, GFP_KERNEL);
		if (bp_panic_buffer)
			memset(bp_panic_buffer, 0, MODEM_PANIC_SIZE);
		bp_panic_ptr =
		    ioremap((int) mot_handover_get_panic(),
			    MODEM_PANIC_RAW_SIZE);
		if (!bp_panic_ptr) {
			printk(KERN_ERR "mpanic_bp_panicdata_dmesg: ioremap failed for bp_panic_ptr!\n");

			kfree(bp_panic_buffer);
			bp_panic_buffer = NULL;
			return;
		}
		printk(KERN_ERR "Modem Panic detected!\n");
		printk(KERN_ERR "\n");

		for (i = 0; i < MODEM_PANIC_RAW_SIZE; i++) {
			/* limit the output to 80 chars/line - account for a
			   timestamp */
			if (i && !(i % 60))
				printk(KERN_CONT "\n");
			printk(KERN_CONT "%02X", bp_panic_ptr[i]);
			if (bp_panic_buffer)
				sprintf(bp_panic_buffer + (i * 2), "%02X",
					(unsigned char) bp_panic_ptr[i]);
		}
		if (bp_panic_buffer)
			bp_panic_buffer[MODEM_PANIC_SIZE - 1] = '\0';
		printk(KERN_ERR "\n");
		printk(KERN_ERR "\n");
	}
}

static void mpanic_proc_entry_create(void)
{
	struct mpanic_data *ctx = &drv_ctx;

	ctx->last_kmsg = create_proc_entry("last_kmsg",
					   S_IFREG | S_IRUGO |
					   S_IWUSR | S_IWGRP, NULL);
	if (!ctx->last_kmsg)
		printk(KERN_ERR "%s: failed creating procfile\n",
		       __func__);
	else {
		ctx->last_kmsg->read_proc = mpanic_proc_last_kmsg_read;
		ctx->last_kmsg->write_proc = mpanic_proc_write;
		if ((unsigned char *) mot_handover_get_panic())
			ctx->last_kmsg->size = bp_panic_size;
		else
			ctx->last_kmsg->size = 0;
		ctx->last_kmsg->data = (void *) 3;

		INIT_WORK(&proc_removal_work, mpanic_remove_proc_work);
	}

	return;
}

static int panic_dbg_get(void *data, u64 * val)
{
	return 0;
}

static int panic_dbg_set(void *data, u64 val)
{
	msm_proc_comm(PCOM_MODEM_PANIC, NULL, NULL);
	return -1;
}

DEFINE_SIMPLE_ATTRIBUTE(panic_dbg_fops, panic_dbg_get, panic_dbg_set,
			"%llu\n");

static int __init mpanic_init(void)
{
	printk(KERN_INFO "mpanic init\n");
	memset(&drv_ctx, 0, sizeof(drv_ctx));
	mpanic_bp_panicdata_dmesg();
	mpanic_proc_entry_create();
	debugfs_create_file("mpanic", 0644, NULL, NULL, &panic_dbg_fops);

	return 0;
}

module_init(mpanic_init);
