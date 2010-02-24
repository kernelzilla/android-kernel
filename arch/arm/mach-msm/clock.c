/* arch/arm/mach-msm/clock.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/seq_file.h>

#include "clock.h"
#include "proc_comm.h"

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static HLIST_HEAD(clocks);

struct clk* axi_clk;  /* hack */

static int clk_set_rate_locked(struct clk *clk, unsigned long rate);

/*
 * glue for the proc_comm interface
 */
static inline int pc_clk_enable(unsigned id)
{
	/* gross hack to set axi clk rate when turning on uartdm clock */
	if (id == UART1DM_CLK && axi_clk)
		clk_set_rate_locked(axi_clk, 128000000);
	return msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, NULL);
}

static inline void pc_clk_disable(unsigned id)
{
	msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, NULL);
	if (id == UART1DM_CLK && axi_clk)
		clk_set_rate_locked(axi_clk, 0);
}

static inline int pc_clk_set_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &rate);
}

static int pc_clk_set_min_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_MIN_RATE, &id, &rate);
}

static inline int pc_clk_set_max_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_MAX_RATE, &id, &rate);
}

static inline int pc_clk_set_flags(unsigned id, unsigned flags)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_SET_FLAGS, &id, &flags);
}

static inline unsigned pc_clk_get_rate(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, NULL))
		return 0;
	else
		return id;
}

static inline unsigned pc_clk_is_enabled(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLED, &id, NULL))
		return 0;
	else
		return id;
}

static inline int pc_pll_request(unsigned id, unsigned on)
{
	on = !!on;
	return msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
}

static struct clk *clk_allocate_handle(struct clk *sclk)
{
	unsigned long flags;
	struct clk_handle *clkh = kzalloc(sizeof(*clkh), GFP_KERNEL);
	if (!clkh)
		return ERR_PTR(ENOMEM);
	clkh->clk.flags = CLKFLAG_HANDLE;
	clkh->source = sclk;

	spin_lock_irqsave(&clocks_lock, flags);
	hlist_add_head(&clkh->clk.list, &sclk->handles);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return &clkh->clk;
}

static struct clk *source_clk(struct clk *clk)
{
	struct clk_handle *clkh;

	if (clk->flags & CLKFLAG_HANDLE) {
		clkh = container_of(clk, struct clk_handle, clk);
		clk = clkh->source;
	}
	return clk;
}

/*
 * Standard clock functions defined in include/linux/clk.h
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;
	struct hlist_node *pos;

	mutex_lock(&clocks_mutex);

	hlist_for_each_entry(clk, pos, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == dev)
			goto found_it;

	hlist_for_each_entry(clk, pos, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == NULL)
			goto found_it;

	clk = ERR_PTR(-ENOENT);
found_it:
	if (!IS_ERR(clk) && (clk->flags & CLKFLAG_SHARED))
		clk = clk_allocate_handle(clk);
	mutex_unlock(&clocks_mutex);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
	struct clk_handle *clkh;
	unsigned long flags;

	if (WARN_ON(IS_ERR(clk)))
		return;

	if (!(clk->flags & CLKFLAG_HANDLE))
		return;

	clk_set_rate(clk, 0);

	spin_lock_irqsave(&clocks_lock, flags);
	clkh = container_of(clk, struct clk_handle, clk);
	hlist_del(&clk->list);
	kfree(clkh);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	clk = source_clk(clk);
	clk->count++;
	if (clk->count == 1)
		pc_clk_enable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	clk = source_clk(clk);
	BUG_ON(clk->count == 0);
	clk->count--;
	if (clk->count == 0)
		pc_clk_disable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	clk = source_clk(clk);
	return pc_clk_get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

static unsigned long clk_find_min_rate_locked(struct clk *clk)
{
	unsigned long rate = 0;
	struct clk_handle *clkh;
	struct hlist_node *pos;

	hlist_for_each_entry(clkh, pos, &clk->handles, clk.list)
		if (clkh->rate > rate)
			rate = clkh->rate;
	return rate;
}

static int clk_set_rate_locked(struct clk *clk, unsigned long rate)
{
	int ret;

	if (clk->flags & CLKFLAG_HANDLE) {
		struct clk_handle *clkh;
		clkh = container_of(clk, struct clk_handle, clk);
		clkh->rate = rate;
		clk = clkh->source;
		rate = clk_find_min_rate_locked(clk);
	}

	if (clk->flags & CLKFLAG_USE_MAX_TO_SET) {
		ret = pc_clk_set_max_rate(clk->id, rate);
		if (ret)
			goto err;
	}
	if (clk->flags & CLKFLAG_USE_MIN_TO_SET) {
		ret = pc_clk_set_min_rate(clk->id, rate);
		if (ret)
			goto err;
	}

	if (!(clk->flags & (CLKFLAG_USE_MAX_TO_SET | CLKFLAG_USE_MIN_TO_SET)))
		ret = pc_clk_set_rate(clk->id, rate);
err:
	return ret;
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	ret = clk_set_rate_locked(clk, rate);
	spin_unlock_irqrestore(&clocks_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	clk = source_clk(clk);
	return pc_clk_set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);

void clk_enter_sleep(int from_idle)
{
}

void clk_exit_sleep(void)
{
}

int clks_print_running(void)
{
	struct clk *clk;
	int clk_on_count = 0;
	struct hlist_node *pos;
	char buf[100];
	char *pbuf = buf;
	int size = sizeof(buf);
	int wr;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);

	hlist_for_each_entry(clk, pos, &clocks, list) {
		if (clk->count) {
			clk_on_count++;
			wr = snprintf(pbuf, size, " %s", clk->name);
			if (wr >= size)
				break;
			pbuf += wr;
			size -= wr;
		}
	}
	if (clk_on_count)
		pr_info("clocks on:%s\n", buf);

	spin_unlock_irqrestore(&clocks_lock, flags);
	return !clk_on_count;
}
EXPORT_SYMBOL(clks_print_running);

void __init msm_clock_init(void)
{
	struct clk *clk;

	spin_lock_init(&clocks_lock);
	mutex_lock(&clocks_mutex);
	for (clk = msm_clocks; clk && clk->name; clk++) {
		hlist_add_head(&clk->list, &clocks);
	}
	mutex_unlock(&clocks_mutex);
}

#if defined(CONFIG_MSM_CLOCK_CTRL_DEBUG)
static int clk_debug_set(void *data, u64 val)
{
	struct clk *clk = data;
	int ret;

	ret = clk_set_rate(clk, val);
	if (ret != 0)
		pr_err("%s: can't set rate of '%s' to %llu (%d)\n",
		       __func__, clk->name, val, ret);
	return ret;
}

static int clk_debug_get(void *data, u64 *val)
{
	*val = clk_get_rate((struct clk *) data);
	return *val == 0;
}

DEFINE_SIMPLE_ATTRIBUTE(clk_debug_fops, clk_debug_get, clk_debug_set, "%llu\n");

static void *clk_info_seq_start(struct seq_file *seq, loff_t *ppos)
{
	struct hlist_node *pos;
	int i = *ppos;
	mutex_lock(&clocks_mutex);
	hlist_for_each(pos, &clocks)
		if (i-- == 0)
			return hlist_entry(pos, struct clk, list);
	return NULL;
}

static void *clk_info_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct clk *clk = v;
	++*pos;
	return hlist_entry(clk->list.next, struct clk, list);
}

static void clk_info_seq_stop(struct seq_file *seq, void *v)
{
	mutex_unlock(&clocks_mutex);
}

static int clk_info_seq_show(struct seq_file *seq, void *v)
{
	struct clk *clk = v;
	unsigned long flags;
	struct clk_handle *clkh;
	struct hlist_node *pos;

	seq_printf(seq, "Clock %s\n", clk->name);
	seq_printf(seq, "  Id          %d\n", clk->id);
	seq_printf(seq, "  Count       %d\n", clk->count);
	seq_printf(seq, "  Flags       %x\n", clk->flags);
	seq_printf(seq, "  Dev         %p %s\n",
			clk->dev, clk->dev ? dev_name(clk->dev) : "");
	seq_printf(seq, "  Handles     %p\n", clk->handles.first);
	spin_lock_irqsave(&clocks_lock, flags);
	hlist_for_each_entry(clkh, pos, &clk->handles, clk.list)
		seq_printf(seq, "    Requested rate    %ld\n", clkh->rate);
	spin_unlock_irqrestore(&clocks_lock, flags);

	seq_printf(seq, "  Enabled     %d\n", pc_clk_is_enabled(clk->id));
	seq_printf(seq, "  Rate        %ld\n", clk_get_rate(clk));

	seq_printf(seq, "\n");
	return 0;
}

static struct seq_operations clk_info_seqops = {
	.start = clk_info_seq_start,
	.next = clk_info_seq_next,
	.stop = clk_info_seq_stop,
	.show = clk_info_seq_show,
};

static int clk_info_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &clk_info_seqops);
}

static const struct file_operations clk_info_fops = {
	.owner = THIS_MODULE,
	.open = clk_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void __init clock_debug_init(void)
{
	struct dentry *dent;
	struct clk *clk;
	struct hlist_node *pos;

	dent = debugfs_create_dir("clk", 0);
	if (IS_ERR(dent)) {
		pr_err("%s: Unable to create debugfs dir (%ld)\n", __func__,
		       PTR_ERR(dent));
		return;
	}

	debugfs_create_file("all", 0x444, dent, NULL, &clk_info_fops);

	mutex_lock(&clocks_mutex);
	hlist_for_each_entry(clk, pos, &clocks, list) {
		debugfs_create_file(clk->name, 0644, dent, clk,
				    &clk_debug_fops);
	}
	mutex_unlock(&clocks_mutex);
}
#else
static inline void __init clock_debug_init(void) {}
#endif


/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	struct hlist_node *pos;
	unsigned count = 0;

	mutex_lock(&clocks_mutex);
	hlist_for_each_entry(clk, pos, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clocks_lock, flags);
			if (!clk->count) {
				count++;
				pc_clk_disable(clk->id);
			}
			spin_unlock_irqrestore(&clocks_lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);

	clock_debug_init();

	axi_clk = clk_get(NULL, "ebi1_clk");

	return 0;
}

late_initcall(clock_late_init);
