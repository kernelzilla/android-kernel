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
#ifndef __GSL_DRAWCTXT_H
#define __GSL_DRAWCTXT_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>

#include "kgsl_sharedmem.h"

struct kgsl_device;

struct kgsl_drawctxt {
	uint32_t         flags;
	struct kgsl_pagetable *pagetable;
	struct kgsl_memdesc       gpustate;
	struct kgsl_memdesc       gmemshadow;
	unsigned int        reg_save[3];
	unsigned int        reg_restore[3];
	unsigned int        gmem_save[3];
	unsigned int        gmem_restore[3];
	unsigned int        shader_save[3];
	unsigned int        shader_fixup[3];
	unsigned int        shader_restore[3];
};


int kgsl_drawctxt_create(struct kgsl_device *, struct kgsl_pagetable *,
			  unsigned int flags,
			  unsigned int *drawctxt_id);

int kgsl_drawctxt_destroy(struct kgsl_device *device, unsigned int drawctxt_id);

int kgsl_drawctxt_init(struct kgsl_device *device);

int kgsl_drawctxt_close(struct kgsl_device *device);

void kgsl_drawctxt_switch(struct kgsl_device *device,
				struct kgsl_drawctxt *drawctxt,
				unsigned int flags);

#endif  /* __GSL_DRAWCTXT_H */
