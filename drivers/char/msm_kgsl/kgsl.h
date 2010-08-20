/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _GSL_DRIVER_H
#define _GSL_DRIVER_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/atomic.h>

#include "kgsl_device.h"
#include "kgsl_sharedmem.h"

#define DRIVER_NAME "kgsl"
#define CHIP_REV_251 0x020501

struct kgsl_driver {
	struct miscdevice misc;
	struct platform_device *pdev;
	atomic_t open_count;
	struct mutex mutex;

	int interrupt_num;
	int have_irq;

	struct clk *grp_pclk;
	struct clk *grp_clk;
	struct clk *imem_clk;
	unsigned int power_flags;
	unsigned int is_suspended;
	unsigned int max_axi_freq;

	struct kgsl_devconfig yamato_config;

	uint32_t flags_debug;

	struct kgsl_sharedmem shmem;
	struct kgsl_device yamato_device;

	struct list_head client_list;
};

extern struct kgsl_driver kgsl_driver;

struct kgsl_pmem_entry {
	struct kgsl_memdesc memdesc;
	struct file *pmem_file;
	struct list_head list;
	struct list_head free_list;
	uint32_t free_timestamp;
};

enum kgsl_status {
	KGSL_SUCCESS = 0,
	KGSL_FAILURE = 1
};

#define KGSL_TRUE 1
#define KGSL_FALSE 0

#define KGSL_PRE_HWACCESS() \
while (1) { \
	mutex_lock(&kgsl_driver.mutex); \
	if (kgsl_driver.yamato_device.hwaccess_blocked == KGSL_FALSE) { \
		break; \
	} \
	if (kgsl_driver.is_suspended != KGSL_TRUE) { \
		kgsl_yamato_wake(&kgsl_driver.yamato_device); \
		break; \
	} \
	mutex_unlock(&kgsl_driver.mutex); \
	wait_for_completion(&kgsl_driver.yamato_device.hwaccess_gate); \
}

#define KGSL_POST_HWACCESS() \
	mutex_unlock(&kgsl_driver.mutex)

void kgsl_remove_pmem_entry(struct kgsl_pmem_entry *entry);

int kgsl_pwrctrl(unsigned int pwrflag);
int kgsl_yamato_sleep(struct kgsl_device *device, const int idle);

#endif /* _GSL_DRIVER_H */
