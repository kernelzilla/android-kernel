/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#ifdef CONFIG_VFP
#include <asm/vfp.h>
#endif

#include "acpuclock.h"
#include "cpuidle.h"
#include "idle.h"
#include "pm.h"
#include "spm.h"
#include "timer.h"

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_PM_DEBUG_SUSPEND = 1U << 0,
	MSM_PM_DEBUG_POWER_COLLAPSE = 1U << 1,
	MSM_PM_DEBUG_CLOCK = 1U << 3,
	MSM_PM_DEBUG_RESET_VECTOR = 1U << 4,
	MSM_PM_DEBUG_IDLE = 1U << 6,
};

static int msm_pm_debug_mask;
module_param_named(
	debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);


/******************************************************************************
 * Sleep Modes and Parameters
 *****************************************************************************/

static unsigned int msm_pm_sleep_mode_mask;
module_param_named(
	sleep_mode_mask, msm_pm_sleep_mode_mask,
	uint, S_IRUGO | S_IWUSR | S_IWGRP
);

static unsigned int msm_pm_idle_sleep_mode_mask;
module_param_named(
	idle_sleep_mode_mask, msm_pm_idle_sleep_mode_mask,
	uint, S_IRUGO | S_IWUSR | S_IWGRP
);

static struct msm_pm_platform_data *msm_pm_modes;

void __init msm_pm_set_platform_data(
	struct msm_pm_platform_data *data, int count)
{
	BUG_ON(MSM_PM_SLEEP_MODE_NR * num_possible_cpus() != count);
	msm_pm_modes = data;
}


/******************************************************************************
 * CONFIG_MSM_IDLE_STATS
 *****************************************************************************/

#ifdef CONFIG_MSM_IDLE_STATS
enum msm_pm_time_stats_id {
	MSM_PM_STAT_REQUESTED_IDLE,
	MSM_PM_STAT_IDLE_WFI,
	MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_POWER_COLLAPSE,
	MSM_PM_STAT_SUSPEND,
	MSM_PM_STAT_COUNT
};

struct msm_pm_time_stats {
	const char *name;
	int64_t first_bucket_time;
	int bucket[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t min_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t max_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int count;
	int64_t total_time;
};

struct msm_pm_cpu_time_stats {
	struct msm_pm_time_stats stats[MSM_PM_STAT_COUNT];
};

static DEFINE_SPINLOCK(msm_pm_stats_lock);
static DEFINE_PER_CPU_SHARED_ALIGNED(
	struct msm_pm_cpu_time_stats, msm_pm_stats);

/*
 * Add the given time data to the statistics collection.
 */
static void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t)
{
	unsigned long flags;
	struct msm_pm_time_stats *stats;
	int64_t bt;
	int i;

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	stats = __get_cpu_var(msm_pm_stats).stats;

	stats[id].total_time += t;
	stats[id].count++;

	bt = t;
	do_div(bt, stats[id].first_bucket_time);

	if (bt < 1ULL << (CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT *
				(CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1)))
		i = DIV_ROUND_UP(fls((uint32_t)bt),
					CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT);
	else
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	stats[id].bucket[i]++;

	if (t < stats[id].min_time[i] || !stats[id].max_time[i])
		stats[id].min_time[i] = t;
	if (t > stats[id].max_time[i])
		stats[id].max_time[i] = t;

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
}

/*
 * Helper function of snprintf where buf is auto-incremented, size is auto-
 * decremented, and there is no return value.
 *
 * NOTE: buf and size must be l-values (e.g. variables)
 */
#define SNPRINTF(buf, size, format, ...) \
	do { \
		if (size > 0) { \
			int ret; \
			ret = snprintf(buf, size, format, ## __VA_ARGS__); \
			if (ret > size) { \
				buf += size; \
				size = 0; \
			} else { \
				buf += ret; \
				size -= ret; \
			} \
		} \
	} while (0)

/*
 * Write out the power management statistics.
 */
static int msm_pm_read_proc
	(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned int cpu = off / MSM_PM_STAT_COUNT;
	int id = off % MSM_PM_STAT_COUNT;
	char *p = page;

	if (count < 1024) {
		*start = (char *) 0;
		*eof = 0;
		return 0;
	}

	if (cpu < num_possible_cpus()) {
		unsigned long flags;
		struct msm_pm_time_stats *stats;
		int i;
		int64_t bucket_time;
		int64_t s;
		uint32_t ns;

		spin_lock_irqsave(&msm_pm_stats_lock, flags);
		stats = per_cpu(msm_pm_stats, cpu).stats;

		s = stats[id].total_time;
		ns = do_div(s, NSEC_PER_SEC);
		SNPRINTF(p, count,
			"[cpu %u] %s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			cpu, stats[id].name,
			stats[id].count,
			s, ns);

		bucket_time = stats[id].first_bucket_time;
		for (i = 0; i < CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1; i++) {
			s = bucket_time;
			ns = do_div(s, NSEC_PER_SEC);
			SNPRINTF(p, count,
				"   <%6lld.%09u: %7d (%lld-%lld)\n",
				s, ns, stats[id].bucket[i],
				stats[id].min_time[i],
				stats[id].max_time[i]);

			bucket_time <<= CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT;
		}

		SNPRINTF(p, count, "  >=%6lld.%09u: %7d (%lld-%lld)\n",
			s, ns, stats[id].bucket[i],
			stats[id].min_time[i],
			stats[id].max_time[i]);

		*start = (char *) 1;
		*eof = (off + 1 >= MSM_PM_STAT_COUNT * num_possible_cpus());

		spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	}

	return p - page;
}
#undef SNPRINTF

#define MSM_PM_STATS_RESET "reset"

/*
 * Reset the power management statistics values.
 */
static int msm_pm_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char buf[sizeof(MSM_PM_STATS_RESET)];
	int ret;
	unsigned long flags;
	unsigned int cpu;

	if (count < strlen(MSM_PM_STATS_RESET)) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, strlen(MSM_PM_STATS_RESET))) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (memcmp(buf, MSM_PM_STATS_RESET, strlen(MSM_PM_STATS_RESET))) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats;
		int i;

		stats = per_cpu(msm_pm_stats, cpu).stats;
		for (i = 0; i < MSM_PM_STAT_COUNT; i++) {
			memset(stats[i].bucket,
				0, sizeof(stats[i].bucket));
			memset(stats[i].min_time,
				0, sizeof(stats[i].min_time));
			memset(stats[i].max_time,
				0, sizeof(stats[i].max_time));
			stats[i].count = 0;
			stats[i].total_time = 0;
		}
	}

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	return count;

write_proc_failed:
	return ret;
}
#undef MSM_PM_STATS_RESET
#endif /* CONFIG_MSM_IDLE_STATS */


/******************************************************************************
 * Configure Hardware before/after Low Power Mode
 *****************************************************************************/

/*
 * Configure hardware registers in preparation for Apps power down.
 */
static void msm_pm_config_hw_before_power_down(void)
{
	return;
}

/*
 * Clear hardware registers after Apps powers up.
 */
static void msm_pm_config_hw_after_power_up(void)
{
	return;
}

/*
 * Configure hardware registers in preparation for SWFI.
 */
static void msm_pm_config_hw_before_swfi(void)
{
	return;
}


/******************************************************************************
 * Suspend Max Sleep Time
 *****************************************************************************/

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
static int msm_pm_sleep_time_override;
module_param_named(sleep_time_override,
	msm_pm_sleep_time_override, int, S_IRUGO | S_IWUSR | S_IWGRP);
#endif

#define SCLK_HZ (32768)
#define MSM_PM_SLEEP_TICK_LIMIT (0x6DDD000)

static uint32_t msm_pm_max_sleep_time;

/*
 * Convert time from nanoseconds to slow clock ticks, then cap it to the
 * specified limit
 */
static int64_t msm_pm_convert_and_cap_time(int64_t time_ns, int64_t limit)
{
	do_div(time_ns, NSEC_PER_SEC / SCLK_HZ);
	return (time_ns > limit) ? limit : time_ns;
}

/*
 * Set the sleep time for suspend.  0 means infinite sleep time.
 */
void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns)
{
	if (max_sleep_time_ns == 0) {
		msm_pm_max_sleep_time = 0;
	} else {
		msm_pm_max_sleep_time = (uint32_t)msm_pm_convert_and_cap_time(
			max_sleep_time_ns, MSM_PM_SLEEP_TICK_LIMIT);

		if (msm_pm_max_sleep_time == 0)
			msm_pm_max_sleep_time = 1;
	}

	if (msm_pm_debug_mask & MSM_PM_DEBUG_SUSPEND)
		pr_info("%s(): Requested %lld ns Giving %u sclk ticks\n",
			__func__, max_sleep_time_ns, msm_pm_max_sleep_time);
}
EXPORT_SYMBOL(msm_pm_set_max_sleep_time);


/******************************************************************************
 *
 *****************************************************************************/

struct msm_pm_device {
	unsigned int cpu;
	uint32_t *reset_vector;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct msm_pm_device, msm_pm_devices);


static void msm_pm_swfi(void)
{
	msm_pm_config_hw_before_swfi();
	msm_arch_idle();
}

static void msm_pm_spm_power_collapse(
	struct msm_pm_device *dev, bool from_idle, bool notify_rpm)
{
	uint32_t saved_vector[2];
	int collapsed = 0;
	int ret;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("%s(): cpu %u, notify_rpm %d\n",
			__func__, dev->cpu, (int) notify_rpm);

	ret = msm_spm_set_low_power_mode(
			MSM_SPM_MODE_POWER_COLLAPSE, notify_rpm);
	WARN_ON(ret);

	saved_vector[0] = dev->reset_vector[0];
	saved_vector[1] = dev->reset_vector[1];
	dev->reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
	dev->reset_vector[1] = virt_to_phys(msm_pm_collapse_exit);

	if (MSM_PM_DEBUG_RESET_VECTOR & msm_pm_debug_mask)
		pr_info("%s(): vector %x %x -> %x %x\n", __func__,
			saved_vector[0], saved_vector[1],
			dev->reset_vector[0], dev->reset_vector[1]);

#ifdef CONFIG_VFP
	if (from_idle)
		vfp_flush_context();
#endif

	collapsed = msm_pm_collapse();

	dev->reset_vector[0] = saved_vector[0];
	dev->reset_vector[1] = saved_vector[1];

	if (collapsed) {
#ifdef CONFIG_VFP
		if (from_idle)
			vfp_reinit();
#endif
		cpu_init();
		local_fiq_enable();
	}

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("%s(): msm_pm_collapse returned, collapsed = %d\n",
			__func__, collapsed);

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);
}

static void msm_pm_power_collapse_standalone(bool from_idle)
{
	struct msm_pm_device *dev = &__get_cpu_var(msm_pm_devices);
	msm_pm_spm_power_collapse(dev, from_idle, false);
}

static void msm_pm_power_collapse(bool from_idle, uint32_t sleep_delay)
{
	struct msm_pm_device *dev = &__get_cpu_var(msm_pm_devices);
	bool notify_rpm = false;
	unsigned long saved_acpuclk_rate;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("%s(): cpu %u, idle %d, delay %u\n",
			__func__, dev->cpu, (int)from_idle, sleep_delay);

	msm_pm_config_hw_before_power_down();
	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("%s(): pre power down\n", __func__);

	saved_acpuclk_rate = acpuclk_power_collapse();
	if (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask)
		pr_info("%s(): change clock rate (old rate = %lu)\n",
			__func__, saved_acpuclk_rate);

	msm_pm_spm_power_collapse(dev, from_idle, notify_rpm);

	if (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask)
		pr_info("%s(): restore clock rate to %lu\n",
			__func__, saved_acpuclk_rate);
	if (acpuclk_set_rate(dev->cpu, saved_acpuclk_rate, SETRATE_PC) < 0)
		pr_err("%s(): failed to restore clock rate(%lu)\n",
			__func__, saved_acpuclk_rate);

	msm_pm_config_hw_after_power_up();
	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("%s(): post power up\n", __func__);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("%s(): return\n", __func__);
}


/******************************************************************************
 * External Idle/Suspend Functions
 *****************************************************************************/

void arch_idle(void)
{
	return;
}

int msm_pm_idle_enter(enum msm_pm_sleep_mode sleep_mode)
{
	int64_t time;
	int64_t timer_expiration;
	bool timer_halted;
#ifdef CONFIG_MSM_IDLE_STATS
	int exit_stat;
#endif

	if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
		pr_info("%s(): cpu %u, mode %d\n",
			__func__, smp_processor_id(), sleep_mode);

	time = ktime_to_ns(ktime_get());
	timer_expiration = 0; /* msm_timer_enter_idle(); */
	timer_halted = false;

	switch (sleep_mode) {
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		msm_pm_swfi();
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_WFI;
#endif
		break;

	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
		msm_pm_power_collapse_standalone(true);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE;
#endif
		break;

	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE: {
		uint32_t sleep_delay;

		sleep_delay = (uint32_t) msm_pm_convert_and_cap_time(
			timer_expiration, MSM_PM_SLEEP_TICK_LIMIT);
		if (sleep_delay == 0) /* 0 would mean infinite time */
			sleep_delay = 1;

		msm_pm_power_collapse(true, sleep_delay);
		timer_halted = true;
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_POWER_COLLAPSE;
#endif
		break;
	}

	default:
		__WARN();
		goto cpuidle_enter_bail_timer;
	}

	/* msm_timer_exit_idle((int) timer_halted); */
	time = ktime_to_ns(ktime_get()) - time;
#ifdef CONFIG_MSM_IDLE_STATS
	msm_pm_add_stat(exit_stat, time);
#endif

	do_div(time, 1000);
	return (int) time;

cpuidle_enter_bail_timer:
	/* msm_timer_exit_idle((int) timer_halted); */
	return 0;
}

static int msm_pm_enter(suspend_state_t state)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	int i;

#ifdef CONFIG_MSM_IDLE_STATS
	int64_t period = 0;
	int64_t time = msm_timer_get_sclk_time(&period);
#endif

	if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
		pr_info("%s()\n", __func__);

	if (smp_processor_id()) {
		__WARN();
		goto enter_exit;
	}

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct msm_pm_platform_data *mode;

		mode = &msm_pm_modes[MSM_PM_MODE(0, i)];
		allow[i] = ((msm_pm_sleep_mode_mask >> i) & 0x1) &&
			mode->supported && mode->suspend_enabled;
	}

	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s(): power collapse\n", __func__);

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
		if (msm_pm_sleep_time_override > 0) {
			int64_t ns = NSEC_PER_SEC *
				(int64_t) msm_pm_sleep_time_override;
			msm_pm_set_max_sleep_time(ns);
			msm_pm_sleep_time_override = 0;
		}
#endif /* CONFIG_MSM_SLEEP_TIME_OVERRIDE */
		msm_pm_power_collapse(false, msm_pm_max_sleep_time);

#ifdef CONFIG_MSM_IDLE_STATS
		if (time != 0) {
			int64_t end_time = msm_timer_get_sclk_time(NULL);
			if (end_time != 0) {
				time = end_time - time;
				if (time < 0)
					time += period;
			} else
				time = 0;
		}

		msm_pm_add_stat(MSM_PM_STAT_SUSPEND, time);
#endif /* CONFIG_MSM_IDLE_STATS */
	} else if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s(): standalone power collapse\n", __func__);
		msm_pm_power_collapse_standalone(false);
	} else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s(): swfi\n", __func__);
		msm_pm_swfi();
	}

enter_exit:
	if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
		pr_info("%s(): return\n", __func__);

	return 0;
}

static struct platform_suspend_ops msm_pm_ops = {
	.enter = msm_pm_enter,
	.valid = suspend_valid_only_mem,
};


/******************************************************************************
 * Initialization routine
 *****************************************************************************/

static int __init msm_pm_init(void)
{
	unsigned int cpu;
#ifdef CONFIG_MSM_IDLE_STATS
	struct proc_dir_entry *d_entry;
#endif

	for_each_possible_cpu(cpu) {
		struct msm_pm_device *dev = &per_cpu(msm_pm_devices, cpu);

		dev->cpu = cpu;
		dev->reset_vector = (uint32_t *) PAGE_OFFSET;
	}

#ifdef CONFIG_MSM_IDLE_STATS
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats =
			per_cpu(msm_pm_stats, cpu).stats;

		stats[MSM_PM_STAT_REQUESTED_IDLE].name = "idle-request";
		stats[MSM_PM_STAT_REQUESTED_IDLE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_WFI].name = "idle-wfi";
		stats[MSM_PM_STAT_IDLE_WFI].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].name =
			"idle-standalone-power-collapse";
		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].
			first_bucket_time = CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].name =
			"idle-power-collapse";
		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_SUSPEND].name = "suspend";
		stats[MSM_PM_STAT_SUSPEND].first_bucket_time =
			CONFIG_MSM_SUSPEND_STATS_FIRST_BUCKET;
	}

	d_entry = create_proc_entry("msm_pm_stats",
			S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	if (d_entry) {
		d_entry->read_proc = msm_pm_read_proc;
		d_entry->write_proc = msm_pm_write_proc;
		d_entry->data = NULL;
	}
#endif  /* CONFIG_MSM_IDLE_STATS */

	suspend_set_ops(&msm_pm_ops);
	msm_cpuidle_init();

	return 0;
}

late_initcall(msm_pm_init);
