/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __GSL_MMU_H
#define __GSL_MMU_H
#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include "kgsl_log.h"
#include "kgsl_sharedmem.h"

#define GSL_PT_SUPER_PTE 8
#define GSL_PT_PAGE_WV		0x00000001
#define GSL_PT_PAGE_RV		0x00000002
#define GSL_PT_PAGE_DIRTY	0x00000004

extern unsigned int kgsl_mmu_enable;

struct kgsl_device;

struct kgsl_mmu_debug {
	unsigned int  config;
	unsigned int  mpu_base;
	unsigned int  mpu_end;
	unsigned int  va_range;
	unsigned int  pt_base;
	unsigned int  page_fault;
	unsigned int  trans_error;
	unsigned int  axi_error;
	unsigned int  interrupt_mask;
	unsigned int  interrupt_status;
};

struct kgsl_ptstats {
	int64_t  maps;
	int64_t  unmaps;
	int64_t  superpteallocs;
	int64_t  superptefrees;
	int64_t  ptswitches;
	int64_t  tlbflushes[KGSL_DEVICE_MAX];
};

struct kgsl_pagetable {
	unsigned int   refcnt;
	struct kgsl_mmu *mmu;
	struct kgsl_memdesc  base;
	uint32_t      va_base;
	unsigned int   va_range;
	unsigned int   last_superpte;
	unsigned int   max_entries;
	struct gen_pool *pool;
};

struct kgsl_mmu {
	unsigned int     refcnt;
	uint32_t      flags;
	struct kgsl_device     *device;
	unsigned int     config;
	uint32_t        mpu_base;
	int              mpu_range;
	uint32_t        va_base;
	unsigned int     va_range;
	struct kgsl_memdesc    dummyspace;
	/* current page table object being used by device mmu */
	struct kgsl_pagetable  *hwpagetable;
	struct kgsl_pagetable  *defaultpagetable;
};


static inline int
kgsl_mmu_isenabled(struct kgsl_mmu *mmu)
{
	return ((mmu)->flags & KGSL_FLAGS_STARTED) ? 1 : 0;
}


int kgsl_mmu_init(struct kgsl_device *device);

int kgsl_mmu_close(struct kgsl_device *device);

struct kgsl_pagetable *kgsl_mmu_createpagetableobject(struct kgsl_mmu *mmu);

int kgsl_mmu_destroypagetableobject(struct kgsl_pagetable *pagetable);

int kgsl_mmu_setpagetable(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable);

int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 unsigned int physaddr,
		 int range,
		 unsigned int protflags,
		 unsigned int *gpuaddr);

int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
					unsigned int gpuaddr, int range);

int kgsl_mmu_querystats(struct kgsl_pagetable *pagetable,
			struct kgsl_ptstats *stats);

void kgsl_mh_intrcallback(struct kgsl_device *device);

#ifdef DEBUG
void kgsl_mmu_debug(struct kgsl_mmu *, struct kgsl_mmu_debug*);
#else
static inline void kgsl_mmu_debug(struct kgsl_mmu *mmu,
				struct kgsl_mmu_debug *mmu_debug)
{

}
#endif /* DEBUG */

#endif /* __GSL_MMU_H */
