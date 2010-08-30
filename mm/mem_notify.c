/*
 * Notify applications of memory pressure via /dev/mem_notify
 *
 * Copyright (C) 2008 Marcelo Tosatti <marcelo@kvack.org>,
 *                    KOSAKI Motohiro <kosaki.motohiro@jp.fujitsu.com>
 *
 * Released under the GPL, see the file COPYING for details.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/vmstat.h>
#include <linux/percpu.h>
#include <linux/timer.h>
#include <linux/mem_notify.h>

#include <asm/atomic.h>

#define MAX_PROC_WAKEUP_GUARD  (10*HZ)
#define MAX_WAKEUP_TASKS (100)

struct mem_notify_file_info {
	unsigned long     last_proc_notify;
	struct file      *file;

	/* for fasync */
	struct list_head  fa_list;
	int               fa_fd;
};

static DECLARE_WAIT_QUEUE_HEAD(mem_wait);
static atomic_long_t nr_under_memory_pressure_zones = ATOMIC_LONG_INIT(0);
static atomic_t nr_watcher_task = ATOMIC_INIT(0);
static LIST_HEAD(mem_notify_fasync_list);
static DEFINE_SPINLOCK(mem_notify_fasync_lock);
static atomic_t nr_fasync_task = ATOMIC_INIT(0);

atomic_long_t last_mem_notify = ATOMIC_LONG_INIT(INITIAL_JIFFIES);

static void mem_notify_kill_fasync_nr(int nr)
{
	struct mem_notify_file_info *iter, *saved_iter;
	LIST_HEAD(l_fired);

	if (!nr)
		return;

	spin_lock(&mem_notify_fasync_lock);

	list_for_each_entry_safe_reverse(iter, saved_iter,
			&mem_notify_fasync_list,
			fa_list) {
		struct fown_struct *fown;

		fown = &iter->file->f_owner;
		send_sigio(fown, iter->fa_fd, POLL_IN);

		list_del(&iter->fa_list);
		list_add(&iter->fa_list, &l_fired);
		if (!--nr)
			break;
	}

	/* rotate moving for FIFO wakeup */
	list_splice(&l_fired, &mem_notify_fasync_list);

	spin_unlock(&mem_notify_fasync_lock);
}

void __memory_pressure_notify(struct zone *zone, int pressure)
{
	int nr_wakeup;
	int flags;
	int nr_poll_wakeup = 0;
	int nr_fasync_wakeup = 0;

	spin_lock_irqsave(&mem_wait.lock, flags);

	if (pressure != zone->mem_notify_status) {
		long val = pressure ? 1 : -1;
		atomic_long_add(val, &nr_under_memory_pressure_zones);
		zone->mem_notify_status = pressure;
	}

	if (pressure) {
		int nr_watcher = atomic_read(&nr_watcher_task);
		int nr_fasync_wait_tasks = atomic_read(&nr_fasync_task);
		int nr_poll_wait_tasks = nr_watcher - nr_fasync_wait_tasks;

		atomic_long_set(&last_mem_notify, jiffies);
		if (!nr_watcher)
			goto out;

		nr_wakeup = (nr_watcher >> 4) + 1;
		if (unlikely(nr_wakeup > MAX_WAKEUP_TASKS))
			nr_wakeup = MAX_WAKEUP_TASKS;

		/*						nr_wakeup
			nr_fasync_wakeup = nr_fasync_wait_taks x ------------
								nr_watcher
		*/
		nr_fasync_wakeup = DIV_ROUND_UP(nr_fasync_wait_tasks *
				nr_wakeup, nr_watcher);
		if (unlikely(nr_fasync_wakeup > nr_fasync_wait_tasks))
			nr_fasync_wakeup = nr_fasync_wait_tasks;

		nr_poll_wakeup = DIV_ROUND_UP(nr_poll_wait_tasks *
				nr_wakeup, nr_watcher);
		if (unlikely(nr_poll_wakeup > nr_poll_wait_tasks))
			nr_poll_wakeup = nr_poll_wait_tasks;

		wake_up_locked_nr(&mem_wait, nr_poll_wakeup);
	}
out:
	spin_unlock_irqrestore(&mem_wait.lock, flags);

	if (nr_fasync_wakeup)
		mem_notify_kill_fasync_nr(nr_fasync_wakeup);
}

static int mem_notify_open(struct inode *inode, struct file *file)
{
	struct mem_notify_file_info *info;
	int    err = 0;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto out;
	}

	info->last_proc_notify = INITIAL_JIFFIES;
	INIT_LIST_HEAD(&info->fa_list);
	info->file = file;
	info->fa_fd = -1;
	file->private_data = info;
	atomic_inc(&nr_watcher_task);
out:
	return err;
}

static int mem_notify_release(struct inode *inode, struct file *file)
{
	struct mem_notify_file_info *info = file->private_data;

	spin_lock(&mem_notify_fasync_lock);
	if (!list_empty(&info->fa_list)) {
		list_del(&info->fa_list);
		atomic_dec(&nr_fasync_task);
	}
	spin_unlock(&mem_notify_fasync_lock);

	kfree(info);
	atomic_dec(&nr_watcher_task);
	return 0;
}

static unsigned int mem_notify_poll(struct file *file, poll_table *wait)
{
	struct mem_notify_file_info *info = file->private_data;
	unsigned long now = jiffies;
	unsigned long timeout;
	unsigned int retval = 0;
	unsigned long guard_time;

	poll_wait_exclusive(file, &mem_wait, wait);

	guard_time = min_t(unsigned long,
			   MEM_NOTIFY_FREQ * atomic_read(&nr_watcher_task),
			   MAX_PROC_WAKEUP_GUARD);
	timeout = info->last_proc_notify + guard_time;
	if (time_before(now, timeout))
		goto out;

	if (atomic_long_read(&nr_under_memory_pressure_zones) != 0) {
		info->last_proc_notify = now;
		retval = POLLIN;
	}

out:
	return retval;
}

static int mem_notify_fasync(int fd, struct file *filp, int on)
{
	struct mem_notify_file_info *info = filp->private_data;
	int result = 0;

	spin_lock(&mem_notify_fasync_lock);
	if (on) {
		if (list_empty(&info->fa_list)) {
			info->fa_fd = fd;
			list_add(&info->fa_list, &mem_notify_fasync_list);
			result = 1;
		} else {
			info->fa_fd = fd;
		}
	} else {
		if (!list_empty(&info->fa_list)) {
			list_del_init(&info->fa_list);
			info->fa_fd = -1;
			result = -1;
		}
	}
	if (result != 0)
		atomic_add(result, &nr_fasync_task);
	spin_unlock(&mem_notify_fasync_lock);
	return abs(result);
}

struct file_operations mem_notify_fops = {
	.open = mem_notify_open,
	.release = mem_notify_release,
	.poll = mem_notify_poll,
	.fasync  = mem_notify_fasync,
};
EXPORT_SYMBOL(mem_notify_fops);
