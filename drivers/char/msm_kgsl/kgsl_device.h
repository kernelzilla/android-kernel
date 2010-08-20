/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
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
#ifndef _KGSL_DEVICE_H
#define _KGSL_DEVICE_H

#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/wait.h>
#include <linux/msm_kgsl.h>

#include <asm/atomic.h>

#include "kgsl_drawctxt.h"
#include "kgsl_mmu.h"
#include "kgsl_ringbuffer.h"

#define KGSL_CONTEXT_MAX        8

#define KGSL_TIMEOUT_NONE       0
#define KGSL_TIMEOUT_DEFAULT    0xFFFFFFFF

#define KGSL_DEV_FLAGS_INITIALIZED0	0x00000001
#define KGSL_DEV_FLAGS_INITIALIZED	0x00000002
#define KGSL_DEV_FLAGS_STARTED		0x00000004
#define KGSL_DEV_FLAGS_ACTIVE		0x00000008

/*****************************************************************************
** power flags
*****************************************************************************/
#define KGSL_PWRFLAGS_POWER_OFF			0x00000001
#define KGSL_PWRFLAGS_POWER_ON			0x00000002
#define KGSL_PWRFLAGS_CLK_ON             0x00000004
#define KGSL_PWRFLAGS_CLK_OFF            0x00000008
#define KGSL_PWRFLAGS_OVERRIDE_ON        0x00000010
#define KGSL_PWRFLAGS_OVERRIDE_OFF       0x00000020

#define KGSL_CHIPID_YAMATODX_REV21  0x20100
#define KGSL_CHIPID_YAMATODX_REV211 0x20101


struct kgsl_device;
struct platform_device;


struct kgsl_memregion {
	unsigned char  *mmio_virt_base;
	unsigned int   mmio_phys_base;
	uint32_t      gpu_base;
	unsigned int   sizebytes;
};

struct kgsl_device {

	unsigned int	  refcnt;
	uint32_t       flags;
	enum kgsl_deviceid    id;
	unsigned int      chip_id;
	struct kgsl_memregion regspace;
	struct kgsl_memdesc memstore;

	struct kgsl_mmu 	  mmu;
	struct kgsl_memregion gmemspace;
	struct kgsl_ringbuffer ringbuffer;
	unsigned int      drawctxt_count;
	struct kgsl_drawctxt *drawctxt_active;
	struct kgsl_drawctxt drawctxt[KGSL_CONTEXT_MAX];
	unsigned int hwaccess_blocked;
	struct completion hwaccess_gate;

	wait_queue_head_t ib1_wq;
};

struct kgsl_devconfig {
	struct kgsl_memregion regspace;

	unsigned int     mmu_config;
	uint32_t        mpu_base;
	int              mpu_range;
	uint32_t        va_base;
	unsigned int     va_range;

	struct kgsl_memregion gmemspace;
};

int kgsl_yamato_start(struct kgsl_device *device, uint32_t flags);

int kgsl_yamato_stop(struct kgsl_device *device);

int kgsl_yamato_idle(struct kgsl_device *device, unsigned int timeout);

int kgsl_yamato_wake(struct kgsl_device *device);

int kgsl_yamato_suspend(struct kgsl_device *device);

int kgsl_yamato_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type, void *value,
				unsigned int sizebytes);

int kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value);

int kgsl_yamato_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value);

int kgsl_yamato_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp, unsigned int timeout);


int kgsl_yamato_init(struct kgsl_device *, struct kgsl_devconfig *);

int kgsl_yamato_close(struct kgsl_device *device);

int kgsl_yamato_runpending(struct kgsl_device *device);

int __init kgsl_yamato_config(struct kgsl_devconfig *,
				struct platform_device *pdev);

int kgsl_yamato_setup_pt(struct kgsl_device *device, struct kgsl_pagetable *);

int kgsl_yamato_cleanup_pt(struct kgsl_device *device, struct kgsl_pagetable *);

int kgsl_yamato_setpagetable(struct kgsl_device *device);

int kgsl_yamato_tlbinvalidate(struct kgsl_device *device);

irqreturn_t kgsl_yamato_isr(int irq, void *data);

#endif  /* _KGSL_DEVICE_H */
