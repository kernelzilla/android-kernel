/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>

#include <mach/smp.h>
#include <mach/hardware.h>
#include <mach/msm_iomap.h>

#define SECONDARY_CPU_WAIT_MS 10

int pen_release = -1;

int get_core_count(void)
{
#ifdef CONFIG_NR_CPUS
	return CONFIG_NR_CPUS;
#else
	return 1;
#endif
}

/* Initialize the present map (cpu_set(i, cpu_present_map)). */
void smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	for (i = 0; i < max_cpus; i++)
		cpu_set(i, cpu_present_map);
}

void smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);
}

/* Executed by primary CPU, brings other CPUs out of reset. Called at boot
   as well as when a CPU is coming out of shutdown induced by echo 0 >
   /sys/devices/.../cpuX.
*/
int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int cnt = 0;
	printk(KERN_DEBUG "Starting secondary CPU %d\n", cpu);

	/* Tell other CPUs to come out or reset.  Note that secondary CPUs
	 * are probably running with caches off, so we'll need to clean to
	 * memory. Normal cache ops will only clean to L2.
	 */
	pen_release = cpu;
	dmac_clean_range((void *)&pen_release,
			 (void *)(&pen_release + sizeof(pen_release)));
	dmac_clean_range((void *)&secondary_data,
			 (void *)(&secondary_data + sizeof(secondary_data)));
	sev();
	dsb();

	/* Wait for done signal. The cpu receiving the signal does not
	 * have the MMU or caching turned on, so all of its reads and
	 * writes are to/from memory.  Need to ensure that when
	 * reading the value we invalidate the cache line so we see the
	 * fresh data from memory as the normal routines may only
	 * invalidate to POU or L1.
	 */
	while (pen_release != 0xFFFFFFFF) {
		dmac_inv_range((void *)&pen_release,
			       (void *)(&pen_release+sizeof(pen_release)));
		msleep_interruptible(1);
		if (cnt++ >= SECONDARY_CPU_WAIT_MS)
			break;
	}

	if (pen_release == 0xFFFFFFFF)
		printk(KERN_DEBUG "Secondary CPU start acked %d\n", cpu);
	else
		printk(KERN_ERR "Secondary CPU failed to start..." \
		       "continuing\n");

	return 0;
}

/* Initialization routine for secondary CPUs after they are brought out of
 * reset.
*/
void platform_secondary_init(unsigned int cpu)
{
	printk(KERN_DEBUG "%s: cpu:%d\n", __func__, cpu);

	trace_hardirqs_off();

	/*
	 * setup GIC (GIC number NOT CPU number and the base address of the
	 * GIC CPU interface
	 */
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);
}
