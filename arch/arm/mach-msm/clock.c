/* arch/arm/mach-msm/clock.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/pm_qos_params.h>
#include <mach/clk.h>

#include "clock.h"
#include "proc_comm.h"

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static DEFINE_SPINLOCK(ebi1_vote_lock);
static LIST_HEAD(clocks);
struct clk *msm_clocks;
unsigned msm_num_clocks;

/*
 * Bitmap of enabled clocks, excluding ACPU which is always
 * enabled
 */
static DECLARE_BITMAP(clock_map_enabled, NR_CLKS);
static DEFINE_SPINLOCK(clock_map_lock);
static struct notifier_block axi_freq_notifier_block;

/*
 * Standard clock functions defined in include/linux/clk.h
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == dev)
			goto found_it;

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == NULL)
			goto found_it;

	clk = ERR_PTR(-ENOENT);
found_it:
	mutex_unlock(&clocks_mutex);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	clk->count++;
	if (clk->count == 1) {
		clk->ops->enable(clk->id);
		spin_lock(&clock_map_lock);
		clock_map_enabled[BIT_WORD(clk->id)] |= BIT_MASK(clk->id);
		spin_unlock(&clock_map_lock);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	BUG_ON(clk->count == 0);
	clk->count--;
	if (clk->count == 0) {
		clk->ops->disable(clk->id);
		spin_lock(&clock_map_lock);
		clock_map_enabled[BIT_WORD(clk->id)] &= ~BIT_MASK(clk->id);
		spin_unlock(&clock_map_lock);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

int clk_reset(struct clk *clk, enum clk_reset_action action)
{
	if (!clk->ops->reset)
		clk->ops->reset = &pc_clk_reset;
	return clk->ops->reset(clk->remote_id, action);
}
EXPORT_SYMBOL(clk_reset);

unsigned long clk_get_rate(struct clk *clk)
{
	return clk->ops->get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->round_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_min_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_min_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_min_rate);

int clk_set_max_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_max_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_max_rate);

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
	return clk->ops->set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);

/* EBI1 is the only shared clock that several clients want to vote on as of
 * this commit. If this changes in the future, then it might be better to
 * make clk_min_rate handle the voting or make ebi1_clk_set_min_rate more
 * generic to support different clocks.
 */
static unsigned long ebi1_min_rate[CLKVOTE_MAX];
static struct clk *ebi1_clk;

/* Rate is in Hz to be consistent with the other clk APIs. */
int ebi1_clk_set_min_rate(enum clkvote_client client, unsigned long rate)
{
	static unsigned long last_set_val = -1;
	unsigned long new_val;
	unsigned long flags;
	int ret = 0, i;

	spin_lock_irqsave(&ebi1_vote_lock, flags);

	ebi1_min_rate[client] = (rate == MSM_AXI_MAX_FREQ) ?
				(clk_get_max_axi_khz() * 1000) : rate;

	new_val = ebi1_min_rate[0];
	for (i = 1; i < CLKVOTE_MAX; i++)
		if (ebi1_min_rate[i] > new_val)
			new_val = ebi1_min_rate[i];

	/* This check is to save a proc_comm call. */
	if (last_set_val != new_val) {
		ret = clk_set_min_rate(ebi1_clk, new_val);
		if (ret < 0) {
			pr_err("Setting EBI1 min rate to %lu Hz failed!\n",
				new_val);
			pr_err("Last successful value was %lu Hz.\n",
				last_set_val);
		} else {
			last_set_val = new_val;
		}
	}

	spin_unlock_irqrestore(&ebi1_vote_lock, flags);

	return ret;
}

static int axi_freq_notifier_handler(struct notifier_block *block,
				unsigned long min_freq, void *v)
{
	/* convert min_freq from KHz to Hz, unless it's a magic value */
	if (min_freq != MSM_AXI_MAX_FREQ)
		min_freq *= 1000;

	return ebi1_clk_set_min_rate(CLKVOTE_PMQOS, min_freq);
}

/*
 * Find out whether any clock is enabled that needs the TCXO clock.
 *
 * On exit, the buffer 'reason' holds a bitmap of ids of all enabled
 * clocks found that require TCXO.
 *
 * reason: buffer to hold the bitmap; must be compatible with
 *         linux/bitmap.h
 * nbits: number of bits that the buffer can hold; 0 is ok
 *
 * Return value:
 *      0: does not require the TCXO clock
 *      1: requires the TCXO clock
 */
int msm_clock_require_tcxo(unsigned long *reason, int nbits)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&clock_map_lock, flags);
	ret = !bitmap_empty(clock_map_enabled, NR_CLKS);
	if (nbits > 0)
		bitmap_copy(reason, clock_map_enabled, min(nbits, NR_CLKS));
	spin_unlock_irqrestore(&clock_map_lock, flags);

	return ret;
}

/*
 * Find the clock matching the given id and copy its name to the
 * provided buffer.
 *
 * Return value:
 * -ENODEV: there is no clock matching the given id
 *       0: success
 */
int msm_clock_get_name(uint32_t id, char *name, uint32_t size)
{
	struct clk *c_clk;
	int ret = -ENODEV;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(c_clk, &clocks, list) {
		if (id == c_clk->id) {
			strlcpy(name, c_clk->name, size);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&clocks_mutex);

	return ret;
}

static unsigned __initdata local_count;

static void __init set_clock_ops(struct clk *clk)
{
	if (!clk->ops) {
		struct clk_ops *ops = clk_7x30_is_local(clk->id);
		if (ops) {
			clk->ops = ops;
			local_count++;
		} else {
			clk->ops = &clk_ops_pcom;
			clk->id = clk->remote_id;
		}
	}
}

void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks)
{
	unsigned n;

	clk_7x30_init();

	spin_lock_init(&clocks_lock);
	mutex_lock(&clocks_mutex);
	msm_clocks = clock_tbl;
	msm_num_clocks = num_clocks;
	for (n = 0; n < msm_num_clocks; n++) {
		set_clock_ops(&msm_clocks[n]);
		list_add_tail(&msm_clocks[n].list, &clocks);
	}
	mutex_unlock(&clocks_mutex);
	if (local_count)
		pr_info("%u clock%s locally owned\n", local_count,
			local_count > 1 ? "s are" : " is");

	ebi1_clk = clk_get(NULL, "ebi1_clk");
	BUG_ON(ebi1_clk == NULL);

	axi_freq_notifier_block.notifier_call = axi_freq_notifier_handler;
	pm_qos_add_notifier(PM_QOS_SYSTEM_BUS_FREQ, &axi_freq_notifier_block);

}

#if defined(CONFIG_DEBUG_FS)
static struct clk *msm_clock_get_nth(unsigned index)
{
	if (index < msm_num_clocks)
		return msm_clocks + index;
	else
		return 0;
}

static int clock_debug_rate_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;

	/* Only increases to max rate will succeed, but that's actually good
	 * for debugging purposes. So we don't check for error. */
	if (clock->flags & CLK_MAX)
		clk_set_max_rate(clock, val);
	if (clock->flags & CLK_MIN)
		ret = clk_set_min_rate(clock, val);
	else
		ret = clk_set_rate(clock, val);
	if (ret != 0)
		printk(KERN_ERR "clk_set%s_rate failed (%d)\n",
			(clock->flags & CLK_MIN) ? "_min" : "", ret);
	return ret;
}

static int clock_debug_rate_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = clk_get_rate(clock);
	return 0;
}

static int clock_debug_enable_set(void *data, u64 val)
{
	struct clk *clock = data;
	int rc = 0;

	if (val)
		rc = clock->ops->enable(clock->id);
	else
		clock->ops->disable(clock->id);

	return rc;
}

static int clock_debug_enable_get(void *data, u64 *val)
{
	struct clk *clock = data;

	*val = clock->ops->is_enabled(clock->id);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_rate_fops, clock_debug_rate_get,
			clock_debug_rate_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_enable_fops, clock_debug_enable_get,
			clock_debug_enable_set, "%llu\n");

static int __init clock_debug_init(void)
{
	struct dentry *dent_rate;
	struct dentry *dent_enable;
	struct clk *clock;
	unsigned n = 0;
	char temp[50], *ptr;

	dent_rate = debugfs_create_dir("clk_rate", 0);
	if (IS_ERR(dent_rate))
		return PTR_ERR(dent_rate);

	dent_enable = debugfs_create_dir("clk_enable", 0);
	if (IS_ERR(dent_enable))
		return PTR_ERR(dent_enable);

	while ((clock = msm_clock_get_nth(n++)) != 0) {
		strncpy(temp, clock->dbg_name, ARRAY_SIZE(temp)-1);
		for (ptr = temp; *ptr; ptr++)
			*ptr = tolower(*ptr);
		debugfs_create_file(temp, 0644, dent_rate,
				    clock, &clock_rate_fops);
		debugfs_create_file(temp, 0644, dent_enable,
				    clock, &clock_enable_fops);
	}
	return 0;
}

device_initcall(clock_debug_init);
#endif

/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	unsigned count = 0;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clocks_lock, flags);
			if (!clk->count) {
				count++;
				clk->ops->auto_off(clk->id);
			}
			spin_unlock_irqrestore(&clocks_lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);
	return 0;
}

late_initcall(clock_late_init);
