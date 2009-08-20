/* arch/arm/mach-omap2/board-sholes-panic.c
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
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define PANIC_PARTITION "kpanic"

#define EXPERIMENTAL_BB_SKIP 1

struct panic_header {
	u32 magic;
#define PANIC_MAGIC 0xdeadf00d

	u32 console_offset;
	u32 console_length;

	u32 threads_offset;
	u32 threads_length;
};

struct sholes_panic_data {
	struct mtd_info		*mtd;
	struct panic_header	curr;
	void			*bounce;
	struct proc_dir_entry	*last_console;
	struct proc_dir_entry	*last_threads;
	struct proc_dir_entry	*last_kmsg;
};

static struct sholes_panic_data drv_ctx;
static struct work_struct proc_removal_work;
static DEFINE_MUTEX(drv_mutex);

static void sholes_panic_erase_callback(struct erase_info *done)
{
	wait_queue_head_t *wait_q = (wait_queue_head_t *) done->priv;
	wake_up(wait_q);
}

static int sholes_panic_proc_read(char *buffer, char **start, off_t offset,
			       int count, int *peof, void *dat)
{
	struct sholes_panic_data *ctx = &drv_ctx;
	size_t file_length;
	off_t file_offset;
	unsigned int page_no;
	off_t page_offset;
	int rc;
	size_t len;

	if (!count)
		return 0;

	mutex_lock(&drv_mutex);
	
	switch ((int) dat) {
	case 1:	/* last_console */
		file_length = ctx->curr.console_length;
		file_offset = ctx->curr.console_offset;
		break;
	case 2:	/* last_threads */
		file_length = ctx->curr.threads_length;
		file_offset = ctx->curr.threads_offset;
		break;
	case 3:	/* last_kmsg */
		file_length = ctx->curr.threads_length + ctx->curr.console_length;
		file_offset = ctx->curr.console_offset;
		break;
	default:
		pr_err("Bad dat (%d)\n", (int) dat);
		mutex_unlock(&drv_mutex);
		return -EINVAL;
	}

	if ((offset + count) > file_length) {
		mutex_unlock(&drv_mutex);
		return 0;
	}

	/* We only support reading a maximum of a flash page */
	if (count > ctx->mtd->writesize)
		count = ctx->mtd->writesize;

	page_no = (file_offset + offset) / ctx->mtd->writesize;
	page_offset = (file_offset + offset) % ctx->mtd->writesize;

	rc = ctx->mtd->read(ctx->mtd,
			    (page_no * ctx->mtd->writesize),
			    ctx->mtd->writesize,
			    &len, ctx->bounce);

	if (page_offset)
		count -= page_offset;
	memcpy(buffer, ctx->bounce + page_offset, count);

	*start = count;

	if ((offset + count) == file_length)
		*peof = 1;
	
	mutex_unlock(&drv_mutex);
	return count;
}

static void mtd_panic_erase(void)
{
	struct sholes_panic_data *ctx = &drv_ctx;
	struct erase_info erase;
	DECLARE_WAITQUEUE(wait, current);
	wait_queue_head_t wait_q;
	int rc, i;

	printk("sholes_panic: Erasing crash partition\n");
	init_waitqueue_head(&wait_q);
	erase.mtd = ctx->mtd;
	erase.callback = sholes_panic_erase_callback;
	erase.len = ctx->mtd->erasesize;
	erase.priv = (u_long)&wait_q;
	for (i = 0; i < ctx->mtd->size; i += ctx->mtd->erasesize) {
		erase.addr = i;
		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&wait_q, &wait);

		rc = ctx->mtd->block_isbad(ctx->mtd, erase.addr);
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

		rc = ctx->mtd->erase(ctx->mtd, &erase);
		if (rc) {
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&wait_q, &wait);
			printk(KERN_ERR
			       "sholes_panic: Erase of region 0x%llx, 0x%llx failed\n",
			       (unsigned long long) erase.addr,
			       (unsigned long long) erase.len);
			if (rc == -EIO) {
				if (ctx->mtd->block_markbad(ctx->mtd, erase.addr)) {
					printk(KERN_ERR
					       "sholes_panic: Err marking "
					       "block bad\n");
					goto out;
				}
				printk(KERN_INFO
				       "sholes_panic: Marked a bad block"
				       " @%llx\n", erase.addr);
				continue;
			}
			goto out;
		}
		schedule();
		remove_wait_queue(&wait_q, &wait);
	}
	printk(KERN_DEBUG "sholes_panic: Panic partition erased\n");
out:
	return;
}

static void sholes_panic_remove_proc_work(struct work_struct *work)
{
	struct sholes_panic_data *ctx = &drv_ctx;

	mutex_lock(&drv_mutex);
	mtd_panic_erase();
	memset(&ctx->curr, 0, sizeof(struct panic_header));
	if (ctx->last_console) {
		remove_proc_entry("last_console", NULL);
		ctx->last_console = NULL;
	}
	if (ctx->last_threads) {
		remove_proc_entry("last_threads", NULL);
		ctx->last_threads = NULL;
	}
	if (ctx->last_kmsg) {
		remove_proc_entry("last_kmsg", NULL);
		ctx->last_kmsg = NULL;
	}
	mutex_unlock(&drv_mutex);
}

static int sholes_panic_proc_write(struct file *file, const char __user *buffer,
				unsigned long count, void *data)
{
	schedule_work(&proc_removal_work);
	return count;
}

static void mtd_panic_notify_add(struct mtd_info *mtd)
{
	struct sholes_panic_data *ctx = &drv_ctx;
	struct panic_header *hdr = ctx->bounce;
	size_t len;
	int rc;

	if (strcmp(mtd->name, PANIC_PARTITION))
		return;

	ctx->mtd = mtd;

	if (mtd->block_isbad(mtd, 0)) {
		printk(KERN_ERR "sholes_panic: Offset 0 bad block. Boourns!\n");
		goto out_err;
	}

	rc = mtd->read(mtd, 0, mtd->writesize, &len, ctx->bounce);
	if (rc && rc == -EBADMSG) {
		printk(KERN_WARNING
		       "sholes_panic: Bad ECC on block 0 (ignored)\n");
	} else if (rc && rc != -EUCLEAN) {
		printk(KERN_ERR "sholes_panic: Error reading block 0 (%d)\n", rc);
		goto out_err;
	}

	if (len != mtd->writesize) {
		printk(KERN_ERR "sholes_panic: Bad read size (%d)\n", rc);
		goto out_err;
	}

	printk(KERN_INFO "Sholes panic driver bound to %s\n", mtd->name);

	if (hdr->magic != PANIC_MAGIC) {
		printk(KERN_INFO "sholes_panic: No panic data available\n");
		return;
	}

	memcpy(&ctx->curr, hdr, sizeof(struct panic_header));

	printk(KERN_INFO "sholes_panic: c(%u, %u) t(%u, %u)\n",
	       hdr->console_offset, hdr->console_length,
	       hdr->threads_offset, hdr->threads_length);

	if (hdr->console_length) {
		ctx->last_console = create_proc_entry("last_console",
						      S_IFREG | S_IRUGO, NULL);
		if (!ctx->last_console)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->last_console->read_proc = sholes_panic_proc_read;
			ctx->last_console->write_proc = sholes_panic_proc_write;
			ctx->last_console->size = hdr->console_length;
			ctx->last_console->data = (void *) 1;
		}
	}

	if (hdr->threads_length) {
		ctx->last_threads = create_proc_entry("last_threads",
						       S_IFREG | S_IRUGO, NULL);
		if (!ctx->last_threads)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->last_threads->read_proc = sholes_panic_proc_read;
			ctx->last_threads->write_proc = sholes_panic_proc_write;
			ctx->last_threads->size = hdr->threads_length;
			ctx->last_threads->data = (void *) 2;
		}
	}

	if (hdr->console_length || hdr->threads_length) {
		ctx->last_kmsg = create_proc_entry("last_kmsg",
						   S_IFREG | S_IRUGO, NULL);
		if (!ctx->last_kmsg)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->last_kmsg->read_proc = sholes_panic_proc_read;
			ctx->last_kmsg->write_proc = sholes_panic_proc_write;
			ctx->last_kmsg->size = hdr->console_length +
					       hdr->threads_length;
			ctx->last_kmsg->data = (void *) 3;
		}
	}
	return;
out_err:
	ctx->mtd = NULL;
}

static void mtd_panic_notify_remove(struct mtd_info *mtd)
{
	struct sholes_panic_data *ctx = &drv_ctx;
	if (mtd == ctx->mtd) {
		ctx->mtd = NULL;
		printk(KERN_INFO "Sholes panic driver unbound from %s\n", mtd->name);
	}
}

static struct mtd_notifier mtd_panic_notifier = {
	.add	= mtd_panic_notify_add,
	.remove	= mtd_panic_notify_remove,
};

static int in_panic = 0;

static int sholes_writeflashpage(struct mtd_info *mtd, loff_t to, const u_char *buf)
{
	int rc;
	size_t wlen;
	int panic = in_interrupt();

	if (panic && !mtd->panic_write) {
		printk(KERN_EMERG "%s: No panic_write available\n", __func__);
		return 0;
	} else if (!panic && !mtd->write) {
		printk(KERN_EMERG "%s: No write available\n", __func__);
		return 0;
	}

	if (panic)
		rc = mtd->panic_write(mtd, to, mtd->writesize, &wlen, buf);
	else
		rc = mtd->write(mtd, to, mtd->writesize, &wlen, buf);

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
static int sholes_panic_write_console(struct mtd_info *mtd, unsigned int off)
{
	struct sholes_panic_data *ctx = &drv_ctx;
	int saved_oip;
	int idx = 0;
	int rc, rc2;
	for (;;) {
		saved_oip = oops_in_progress;
		oops_in_progress = 1;
		rc = log_buf_copy(ctx->bounce, idx, mtd->writesize);
		oops_in_progress = saved_oip;
		if (rc <= 0)
			break;
		if (rc != mtd->writesize)
			memset(ctx->bounce + rc, 0, mtd->writesize - rc);
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
			       "block @%x\n", off);
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

		rc2 = sholes_writeflashpage(mtd, off, ctx->bounce);
		if (rc2 <= 0) {
			printk(KERN_EMERG
			       "sholes_panic: Flash write failed (%d)\n", rc2);
			return rc2;
		}
		idx += rc2;
		off += rc2;
	}
	return idx;
}

static int sholes_panic(struct notifier_block *this, unsigned long event,
			void *ptr)
{
	struct sholes_panic_data *ctx = &drv_ctx;
	struct panic_header *hdr = (struct panic_header *) ctx->bounce;
	int console_offset = 0;
	int console_len = 0;
	int threads_offset = 0;
	int threads_len = 0;
	int rc;

	if (in_panic)
		return NOTIFY_DONE;
	in_panic = 1;

	if (!ctx->mtd)
		goto out;

	console_offset = ctx->mtd->writesize;

	/*
	 * Write out the console
	 */
	console_len = sholes_panic_write_console(ctx->mtd, console_offset);
	if (console_len < 0) {
		printk(KERN_EMERG "Error writing console to panic log! (%d)\n",
		       console_len);
		console_len = 0;
	}

	/*
	 * Write out all threads
	 */
	threads_offset = ALIGN(console_offset + console_len,
			       ctx->mtd->writesize);
	log_buf_clear();
	show_state_filter(0);
	threads_len = sholes_panic_write_console(ctx->mtd, threads_offset);
	if (threads_len < 0) {
		printk(KERN_EMERG "Error writing threads to panic log! (%d)\n",
		       threads_len);
		threads_len = 0;
	}

	/*
	 * Finally write the panic header
	 */
	memset(ctx->bounce, 0, PAGE_SIZE);
	hdr->magic = PANIC_MAGIC;

	hdr->console_offset = console_offset;
	hdr->console_length = console_len;

	hdr->threads_offset = threads_offset;
	hdr->threads_length = threads_len;

	rc = sholes_writeflashpage(ctx->mtd, 0, ctx->bounce);
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
	memset(&drv_ctx, 0, sizeof(drv_ctx));
	drv_ctx.bounce = (void *) __get_free_page(GFP_KERNEL);
        INIT_WORK(&proc_removal_work, sholes_panic_remove_proc_work);
}

