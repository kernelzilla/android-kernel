/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef _KGSL_DRM_H_
#define _KGSL_DRM_H_

#include <linux/platform_device.h>

#include "drm.h"
#include "drmP.h"

#ifdef CONFIG_MSM_KGSL_DRM

#define DRM_KGSL_GEM_CREATE 0x00
#define DRM_KGSL_GEM_PREP   0x01

#define DRM_IOCTL_KGSL_GEM_CREATE \
DRM_IOWR(DRM_COMMAND_BASE + DRM_KGSL_GEM_CREATE, struct drm_kgsl_gem_create)

#define DRM_IOCTL_KGSL_GEM_PREP \
DRM_IOWR(DRM_COMMAND_BASE + DRM_KGSL_GEM_PREP, struct drm_kgsl_gem_prep)

struct drm_kgsl_gem_object {
	struct drm_gem_object *obj;
	uint32_t pmem_phys;
	uint64_t mmap_offset;
};

struct drm_kgsl_gem_create {
	uint32_t size;
	uint32_t handle;
};

struct drm_kgsl_gem_prep {
	uint32_t handle;
	uint32_t phys;
	uint64_t offset;
};

extern int kgsl_drm_init(struct platform_device *dev);
extern void kgsl_drm_exit(void);
#else
static inline int kgsl_drm_init(struct platform_device *dev)
{
	return 0;
}

static inline void kgsl_drm_exit(void)
{
}
#endif

#endif
