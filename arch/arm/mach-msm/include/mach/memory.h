/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#ifdef CONFIG_MSM_STACKED_MEMORY

#ifdef CONFIG_ARCH_MSM_SCORPION
#define PHYS_OFFSET		UL(0x20000000)
#else
#define PHYS_OFFSET		UL(0x10000000)
#endif

#else /* !CONFIG_MSM_STACKED_MEMORY */

#define PHYS_OFFSET		UL(0x00200000)

#endif

#define HAS_ARCH_IO_REMAP_PFN_RANGE

#define CONSISTENT_DMA_SIZE (4*SZ_1M)

#ifndef __ASSEMBLY__
void *alloc_bootmem_aligned(unsigned long size, unsigned long alignment);
void clean_and_invalidate_caches(unsigned long, unsigned long, unsigned long);
void clean_caches(unsigned long, unsigned long, unsigned long);
void invalidate_caches(unsigned long, unsigned long, unsigned long);

#ifdef CONFIG_ARCH_MSM_ARM11
void write_to_strongly_ordered_memory(void);

#include <asm/mach-types.h>

#define arch_barrier_extra() do \
	{ if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa())  \
		write_to_strongly_ordered_memory(); \
	} while (0)
#endif
#endif

#ifdef CONFIG_ARCH_MSM_SCORPION
#define arch_has_speculative_dfetch()	1
#endif

#endif
