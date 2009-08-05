/* arch/arm/mach-msm/smd_rpcrouter.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
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
#include <linux/spinlock.h>

#define PANIC_PARTITION "kpanic"

#define EXPERIMENTAL_BB_SKIP 0

struct panic_header {
	u32 magic;
#define PANIC_MAGIC 0xdeadf00d

	u32 console_offset;
	u32 console_length;

	u32 threads_offset;
	u32 threads_length;

};

static struct mtd_info *panic_mtd_dev = NULL;
static void *mtd_bounce_page = NULL;

static DEFINE_SPINLOCK(kmsg_lock);
static void *kmsg_data = NULL;
static unsigned kmsg_data_len = 0;

static void sholes_panic_erase_callback(struct erase_info *done)
{
	wait_queue_head_t *wait_q = (wait_queue_head_t *) done->priv;
	wake_up(wait_q);
}

static ssize_t sholes_panic_proc_read(struct file *file, char __user *buf,
				     size_t len, loff_t *offset)
{
	unsigned long flags;
	loff_t pos = *offset;
	ssize_t count;

	spin_lock_irqsave(&kmsg_lock, flags);
	if (pos >= kmsg_data_len)
		return 0;

	count = min(len, (size_t)(kmsg_data_len - pos));
	if (copy_to_user(buf, kmsg_data + pos, count)) {
		spin_unlock_irqrestore(&kmsg_lock, flags);
		return -EFAULT;
	}
	*offset += count;
	spin_unlock_irqrestore(&kmsg_lock, flags);
	return count;
}

static ssize_t sholes_panic_proc_write(struct file *file,
				       const char __user *buf,
				       size_t len, loff_t *offset)
{
	unsigned long flags;
	spin_lock_irqsave(&kmsg_lock, flags);
	if (kmsg_data) {
		kfree(kmsg_data);
		kmsg_data = NULL;
		kmsg_data_len = 0;
		printk(KERN_INFO "sholes-panic: kmsg buffer freed\n");
	}
	spin_unlock_irqrestore(&kmsg_lock, flags);
	return len;
}

static const struct file_operations sholes_panic_file_ops = {
	.owner = THIS_MODULE,
	.read = sholes_panic_proc_read,
	.write = sholes_panic_proc_write,
};

static void mtd_panic_notify_add(struct mtd_info *mtd)
{
	struct proc_dir_entry *entry;
	struct panic_header *hdr = mtd_bounce_page;
	size_t len;
	struct erase_info erase;
	DECLARE_WAITQUEUE(wait, current);
	wait_queue_head_t wait_q;
	int rc, i;

	if (strcmp(mtd->name, PANIC_PARTITION))
		return;

	panic_mtd_dev = mtd;

	/* If theres a dump currently in flash, suck it out */
	if (!mtd->read) {
		printk(KERN_ERR "sholes_panic: No read available on mtd\n");
		return;
	}

	if (mtd->block_isbad(mtd, 0)) {
		printk(KERN_ERR "sholes_panic: Offset 0 bad block. Boourns!\n");
		panic_mtd_dev = NULL;
		return;
	}

	rc = mtd->read(mtd, 0, PAGE_SIZE, &len, mtd_bounce_page);
	if (rc) {
		printk(KERN_ERR "sholes_panic: Err reading flash (%d)\n", rc);
		return;
	}
	if (len != PAGE_SIZE) {
		printk(KERN_ERR "sholes_panic: Bad read size (%d)\n", rc);
		return;
	}

	printk(KERN_INFO "Sholes panic driver bound to %s\n", mtd->name);

	if (hdr->magic != PANIC_MAGIC) {
		printk(KERN_INFO "sholes_panic: No panic data available\n");
		goto erase;
	}

	printk(KERN_INFO "sholes_panic: c(%u, %u) t(%u, %u)\n",
	       hdr->console_offset, hdr->console_length,
	       hdr->threads_offset, hdr->threads_length);

	entry = create_proc_entry("last_kmsg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "sholes_panic: failed to create proc entry\n");
		return;
	}
	entry->proc_fops = &sholes_panic_file_ops;
	entry->size = hdr->console_length + hdr->threads_length;

	kmsg_data = kmalloc(ALIGN(entry->size, PAGE_SIZE), GFP_KERNEL);
	if (!kmsg_data) {
		printk(KERN_ERR "sholes_panic: failed to alloc buffer\n");
		remove_proc_entry("last_kmsg", NULL);
		return;
	}
	memset(kmsg_data, 0, ALIGN(entry->size, PAGE_SIZE));
	printk("sholes_panic: Allocated %llu bytes for kmsg buffer\n",
	       (ALIGN(entry->size, PAGE_SIZE)));

	for (i = 0; i < hdr->console_length; i += PAGE_SIZE) {
		rc = mtd->read(mtd,
			       i + hdr->console_offset,
			       PAGE_SIZE, &len, kmsg_data + i);
		if (rc) {
			printk(KERN_ERR "sholes_panic: Err reading console\n");
			remove_proc_entry("last_kmsg", NULL);
			return;
		}
	}
	kmsg_data_len = hdr->console_length;

	for (i = 0; i < hdr->threads_length; i += PAGE_SIZE) {
		rc = mtd->read(mtd,
			       i + hdr->threads_offset,
			       PAGE_SIZE, &len, kmsg_data + kmsg_data_len + i);
		if (rc) {
			printk(KERN_ERR "sholes_panic: Err reading threads\n");
			remove_proc_entry("last_kmsg", NULL);
			return;
		}
	}
	kmsg_data_len += hdr->threads_length;

erase:

	init_waitqueue_head(&wait_q);
	erase.mtd = mtd;
	erase.callback = sholes_panic_erase_callback;
	erase.len = mtd->erasesize;
	erase.priv = (u_long)&wait_q;
	for (i = 0; i < mtd->size; i += mtd->erasesize) {
		erase.addr = i;
		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&wait_q, &wait);

		rc = mtd->block_isbad(mtd, erase.addr);
		if (rc < 0) {
			printk(KERN_ERR
			       "sholes_panic: Bad block check "
			       "failed (%d)\n", rc);
			goto out;
		}
		if (rc) {
			printk(KERN_WARNING
			       "sholes_panic: Skipping erase of bad "
			       "block @%llx\n", erase.addr);
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&wait_q, &wait);
			continue;
		}

		rc = mtd->erase(mtd, &erase);
		if (rc) {
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&wait_q, &wait);
			printk(KERN_ERR
			       "sholes_panic: Erase of region 0x%llx, 0x%llx failed\n",
			       (unsigned long long) erase.addr,
			       (unsigned long long) erase.len);
			goto out;
		}
		schedule();
		remove_wait_queue(&wait_q, &wait);
	}
	printk(KERN_DEBUG "sholes_panic: Panic partition cleared\n");
out:
	return;
}

static void mtd_panic_notify_remove(struct mtd_info *mtd)
{
	if (mtd == panic_mtd_dev)
		panic_mtd_dev = NULL;
	printk(KERN_INFO "Sholes panic driver unbound from %s\n", mtd->name);
}

static struct mtd_notifier mtd_panic_notifier = {
	.add	= mtd_panic_notify_add,
	.remove	= mtd_panic_notify_remove,
};

static int in_panic = 0;

static int sholes_writeflash(struct mtd_info *mtd, loff_t to, size_t num_pages,
			     const u_char *buf, int panic)
{
	int rc;
	size_t wlen;

	panic = 0; /* omap2 write is safe */

	if (panic && !mtd->panic_write) {
		printk(KERN_EMERG "%s: No panic_write available\n", __func__);
		return 0;
	} else if (!panic && !mtd->write) {
		printk(KERN_EMERG "%s: No write available\n", __func__);
		return 0;
	}

	if (panic)
		rc = mtd->panic_write(mtd, to, PAGE_SIZE, &wlen, buf);
	else
		rc = mtd->write(mtd, to, PAGE_SIZE, &wlen, buf);

	if (rc) {
		printk(KERN_EMERG
		       "%s: Error writing data to flash (%d)\n",
		       __func__, rc);
		return rc;
	}

	return wlen;
}

extern int log_buf_copy(char *dest, int idx, int len);
extern void log_buf_clear(void);

/*
 * Writes the contents of the console to the specified offset in flash.
 * Returns number of bytes written
 */
static int sholes_panic_write_console(struct mtd_info *mtd, unsigned int off,
				      int panic)
{
	int saved_oip;
	int idx = 0;
	int rc, rc2;
	for (;;) {
		saved_oip = oops_in_progress;
		oops_in_progress = 1;
		rc = log_buf_copy(mtd_bounce_page, idx, PAGE_SIZE);
		oops_in_progress = saved_oip;
		if (rc <= 0)
			break;
		if (rc != PAGE_SIZE)
			memset(mtd_bounce_page + rc, 0, PAGE_SIZE - rc);
#if EXPERIMENTAL_BB_SKIP
check_badblock:
		rc = mtd->block_isbad(mtd, off);
		if (rc < 0) {
			printk(KERN_ERR
			       "sholes_panic: Bad block check "
			       "failed (%d)\n", rc);
		}
		if (rc) {
			printk(KERN_WARNING
			       "sholes_panic: Skipping over bad "
			       "block @%llx\n", off);
			off += mtd->erasesize;
			printk("chk %u %llu\n", off, mtd->size);
			if (off >= mtd->size) {
				printk(KERN_EMERG
				       "sholes-panic: Too many bad "
				       "blocks!\n");
				       return -EIO;
			}
			goto check_badblock;
		}
#endif /* EXPERIMENTAL_BB_SKIP */

		rc2 = sholes_writeflash(mtd, off, 1, mtd_bounce_page, panic);
		if (rc2 <= 0) {
			printk(KERN_EMERG
			       "sholes_panic: Flash write failed (%d)\n", rc2);
			return rc2;
		}
		idx += rc;
		off += rc;
	}
	return idx;
}

static int sholes_panic(struct notifier_block *this, unsigned long event,
			void *ptr)
{
	struct panic_header *hdr = (struct panic_header *) mtd_bounce_page;
	int sys_panic = in_interrupt();
	int console_len;
	int threads_offset;
	int threads_len;
	int rc;

	if (in_panic)
		return NOTIFY_DONE;
	in_panic = 1;

	if (!panic_mtd_dev)
		goto out;


	/*
	 * Write out the console
	 */
	console_len = sholes_panic_write_console(panic_mtd_dev, PAGE_SIZE,
						 sys_panic);
	if (console_len < 0) {
		printk(KERN_EMERG "Error writing console to panic log! (%d)\n",
		       console_len);
		console_len = 0;
	}

	/*
	 * Write out all threads
	 */
	threads_offset = ALIGN(PAGE_SIZE + console_len, PAGE_SIZE);
	log_buf_clear();
	show_state_filter(0);
	threads_len = sholes_panic_write_console(panic_mtd_dev, threads_offset,
						 sys_panic);
	if (threads_len < 0) {
		printk(KERN_EMERG "Error writing threads to panic log! (%d)\n",
		       threads_len);
		threads_len = 0;
	}

	/*
	 * Finally write the panic header
	 */
	memset(mtd_bounce_page, 0, PAGE_SIZE);
	hdr->magic = PANIC_MAGIC;

	hdr->console_offset = PAGE_SIZE;
	hdr->console_length = console_len;

	hdr->threads_offset = threads_offset;
	hdr->threads_length = threads_len;

	rc = sholes_writeflash(panic_mtd_dev, 0, 1, mtd_bounce_page, sys_panic);
	if (rc <= 0) {
		printk(KERN_EMERG "sholes_panic: Header write failed (%d)\n",
		       rc);
		goto out;
	}

	printk(KERN_EMERG "sholes_panic: Panic dump sucessfully written to flash\n");

 out:
	in_panic = 0;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= sholes_panic,
};

static int panic_dbg_get(void *data, u64 *val)
{
	sholes_panic(NULL, 0, NULL);
	return 0;
}

static int panic_dbg_set(void *data, u64 val)
{
	BUG();
	panic("%s: test panic!\n", __func__);
}

DEFINE_SIMPLE_ATTRIBUTE(panic_dbg_fops, panic_dbg_get, panic_dbg_set, "%llu\n");

void __init sholes_panic_init(void)
{
	register_mtd_user(&mtd_panic_notifier);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	debugfs_create_file("panic", 0644, NULL, NULL, &panic_dbg_fops);
	mtd_bounce_page = __get_free_page(GFP_KERNEL);
}

