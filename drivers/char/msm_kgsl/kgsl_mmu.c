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
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>

#include "kgsl_mmu.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "yamato_reg.h"

struct kgsl_pte_debug {
	unsigned int read:1;
	unsigned int write:1;
	unsigned int dirty:1;
	unsigned int reserved:9;
	unsigned int phyaddr:20;
};

#define GSL_PTE_SIZE	4
#define GSL_PT_EXTRA_ENTRIES	16


#define GSL_PT_PAGE_BITS_MASK	0x00000007
#define GSL_PT_PAGE_ADDR_MASK	(~(KGSL_PAGESIZE - 1))

#define GSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR)

uint32_t kgsl_pt_entry_get(struct kgsl_pagetable *pt, uint32_t va)
{
	return (va - pt->va_base) >> KGSL_PAGESIZE_SHIFT;
}

uint32_t kgsl_pt_map_get(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte];
}

void kgsl_pt_map_set(struct kgsl_pagetable *pt, uint32_t pte, uint32_t val)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] = val;
}
#define GSL_PT_MAP_DEBUG(pte)	((struct kgsl_pte_debug *) \
		&gsl_pt_map_get(pagetable, pte))

void kgsl_pt_map_setbits(struct kgsl_pagetable *pt, uint32_t pte, uint32_t bits)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] |= bits;
}

void kgsl_pt_map_setaddr(struct kgsl_pagetable *pt, uint32_t pte,
					uint32_t pageaddr)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	uint32_t val = baseptr[pte];
	val &= ~GSL_PT_PAGE_ADDR_MASK;
	val |= (pageaddr & GSL_PT_PAGE_ADDR_MASK);
	baseptr[pte] = val;
}

void kgsl_pt_map_resetall(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] &= GSL_PT_PAGE_DIRTY;
}

void kgsl_pt_map_resetbits(struct kgsl_pagetable *pt, uint32_t pte,
				uint32_t bits)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] &= ~(bits & GSL_PT_PAGE_BITS_MASK);
}

int kgsl_pt_map_isdirty(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte] & GSL_PT_PAGE_DIRTY;
}

uint32_t kgsl_pt_map_getaddr(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte] & GSL_PT_PAGE_ADDR_MASK;
}

void kgsl_mh_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int reg;
	struct kgsl_mmu_debug dbg;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_MH_INTERRUPT_STATUS, &status);

	if (status & MH_INTERRUPT_MASK__AXI_READ_ERROR) {
		KGSL_MEM_FATAL("axi read error interrupt\n");
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__AXI_WRITE_ERROR) {
		KGSL_MEM_FATAL("axi write error interrupt\n");
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__MMU_PAGE_FAULT) {
		kgsl_yamato_regread(device, REG_MH_MMU_PAGE_FAULT, &reg);
		KGSL_MEM_FATAL("mmu page fault interrupt: %08x\n", reg);
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else {
		KGSL_MEM_DBG("bad bits in REG_MH_INTERRUPT_STATUS %08x\n",
			     status);
	}

	kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_CLEAR, status);

	/*TODO: figure out how to handle errror interupts.
	* specifically, page faults should probably nuke the client that
	* caused them, but we don't have enough info to figure that out yet.
	*/

	KGSL_MEM_VDBG("return\n");
}

#ifdef DEBUG
void kgsl_mmu_debug(struct kgsl_mmu *mmu, struct kgsl_mmu_debug *regs)
{
	memset(regs, 0, sizeof(struct kgsl_mmu_debug));

	kgsl_yamato_regread(mmu->device, REG_MH_MMU_CONFIG,
			    &regs->config);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_MPU_BASE,
			    &regs->mpu_base);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_MPU_END,
			    &regs->mpu_end);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_VA_RANGE,
			    &regs->va_range);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_PT_BASE,
			    &regs->pt_base);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_PAGE_FAULT,
			    &regs->page_fault);
	kgsl_yamato_regread(mmu->device, REG_MH_MMU_TRAN_ERROR,
			    &regs->trans_error);
	kgsl_yamato_regread(mmu->device, REG_MH_AXI_ERROR,
			    &regs->axi_error);
	kgsl_yamato_regread(mmu->device, REG_MH_INTERRUPT_MASK,
			    &regs->interrupt_mask);
	kgsl_yamato_regread(mmu->device, REG_MH_INTERRUPT_STATUS,
			    &regs->interrupt_status);

	KGSL_MEM_DBG("mmu config %08x mpu_base %08x mpu_end %08x\n",
		     regs->config, regs->mpu_base, regs->mpu_end);
	KGSL_MEM_DBG("mmu va_range %08x pt_base %08x \n",
		     regs->va_range, regs->pt_base);
	KGSL_MEM_DBG("mmu page_fault %08x tran_err %08x\n",
		     regs->page_fault, regs->trans_error);
	KGSL_MEM_DBG("mmu int mask %08x int status %08x\n",
			regs->interrupt_mask, regs->interrupt_status);
}
#endif

struct kgsl_pagetable *kgsl_mmu_createpagetableobject(struct kgsl_mmu *mmu)
{
	int status = 0;
	struct kgsl_pagetable *pagetable = NULL;
	uint32_t flags;

	KGSL_MEM_VDBG("enter (mmu=%p)\n", mmu);

	pagetable = kzalloc(sizeof(struct kgsl_pagetable), GFP_KERNEL);
	if (pagetable == NULL) {
		KGSL_MEM_ERR("Unable to allocate pagetable object.\n");
		return NULL;
	}

	pagetable->mmu = mmu;
	pagetable->va_base = mmu->va_base;
	pagetable->va_range = mmu->va_range;
	pagetable->last_superpte = 0;
	pagetable->max_entries = (mmu->va_range >> KGSL_PAGESIZE_SHIFT)
				 + GSL_PT_EXTRA_ENTRIES;

	pagetable->pool = gen_pool_create(KGSL_PAGESIZE_SHIFT, -1);
	if (pagetable->pool == NULL) {
		KGSL_MEM_ERR("Unable to allocate virtualaddr pool.\n");
		return NULL;
	}

	if (gen_pool_add(pagetable->pool, pagetable->va_base,
				pagetable->va_range, -1)) {
		KGSL_MEM_ERR("gen_pool_create failed for pagetable %p\n",
				pagetable);
		goto done;
	}

	/* allocate page table memory */
	flags = (KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS
		 | KGSL_MEMFLAGS_STRICTREQUEST);
	status = kgsl_sharedmem_alloc(flags,
				      pagetable->max_entries * GSL_PTE_SIZE,
				      &pagetable->base);

	if (status == 0) {
		/* reset page table entries
		 * -- all pte's are marked as not dirty initially
		 */
		pagetable->base.gpuaddr = pagetable->base.physaddr;
		kgsl_sharedmem_set(&pagetable->base, 0, 0,
				   pagetable->base.size);
	}

	KGSL_MEM_VDBG("return %p\n", pagetable);

	return pagetable;
done:
	return NULL;
}

int kgsl_mmu_destroypagetableobject(struct kgsl_pagetable *pagetable)
{
	KGSL_MEM_VDBG("enter (pagetable=%p)\n", pagetable);

	if (pagetable) {
		if (pagetable->base.gpuaddr)
			kgsl_sharedmem_free(&pagetable->base);

		if (pagetable->pool) {
			gen_pool_destroy(pagetable->pool);
			pagetable->pool = NULL;
		}

		kfree(pagetable);

	}
	KGSL_MEM_VDBG("return 0x%08x\n", 0);

	return 0;
}

int kgsl_mmu_setpagetable(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable)
{
	int status = 0;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p, pagetable=%p)\n", device, pagetable);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* page table not current, then setup mmu to use new
		 *  specified page table
		 */
		KGSL_MEM_INFO("from %p to %p\n", mmu->hwpagetable, pagetable);
		if (mmu->hwpagetable != pagetable) {
			mmu->hwpagetable = pagetable;

			/* call device specific set page table */
			status = kgsl_yamato_setpagetable(mmu->device);
		}
	}

	KGSL_MEM_VDBG("return %d\n", status);

	return status;
}

int kgsl_mmu_init(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	uint32_t flags;
	struct kgsl_pagetable *pagetable = NULL;
	struct kgsl_mmu *mmu = &device->mmu;
#ifdef _DEBUG
	struct kgsl_mmu_debug regs;
#endif /* _DEBUG */

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		KGSL_MEM_INFO("MMU already initialized.\n");
		return 0;
	}

	mmu->device = device;

	if (!kgsl_mmu_enable)
		mmu->config = 0x00000000;

	/* setup MMU and sub-client behavior */
	kgsl_yamato_regwrite(device, REG_MH_MMU_CONFIG, mmu->config);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK);
	kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_MASK, GSL_MMU_INT_MASK);

	mmu->flags |= KGSL_FLAGS_INITIALIZED0;

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	/* idle device */
	kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

	/* make sure aligned to pagesize */
	BUG_ON(mmu->mpu_base & (KGSL_PAGESIZE - 1));
	BUG_ON((mmu->mpu_base + mmu->mpu_range) & (KGSL_PAGESIZE - 1));

	/* define physical memory range accessible by the core */
	kgsl_yamato_regwrite(device, REG_MH_MMU_MPU_BASE,
				mmu->mpu_base);
	kgsl_yamato_regwrite(device, REG_MH_MMU_MPU_END,
				mmu->mpu_base + mmu->mpu_range);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);
	kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_MASK,
			GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);

	mmu->flags |= KGSL_FLAGS_INITIALIZED;

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {
		/*make sure virtual address range is a multiple of 64Kb */
		BUG_ON(mmu->va_range & ((1 << 16) - 1));

		/* setup pagetable object */
		pagetable = kgsl_mmu_createpagetableobject(mmu);
		if (!pagetable) {
			kgsl_mmu_close(device);
			KGSL_MEM_VDBG("return %d\n", -EINVAL);
			return -EINVAL;
		}

		mmu->hwpagetable = pagetable;
		mmu->defaultpagetable = pagetable;
		/* set page table base */
		kgsl_yamato_regwrite(device, REG_MH_MMU_PT_BASE,
				     mmu->hwpagetable->base.gpuaddr);

		/* define virtual address range */
		kgsl_yamato_regwrite(device, REG_MH_MMU_VA_RANGE,
				     (mmu->va_base | (mmu->va_range >> 16)));

		/* allocate memory used for completing r/w operations that
		 * cannot be mapped by the MMU
		 */
		flags = (KGSL_MEMFLAGS_ALIGN32 | KGSL_MEMFLAGS_CONPHYS
			 | KGSL_MEMFLAGS_STRICTREQUEST);
		status = kgsl_sharedmem_alloc(flags, 32, &mmu->dummyspace);
		if (status != 0) {
			KGSL_MEM_ERR
			    ("Unable to allocate dummy space memory.\n");
			kgsl_mmu_close(device);
			return status;
		}
		mmu->dummyspace.gpuaddr = mmu->dummyspace.physaddr;

		kgsl_yamato_regwrite(device,
				     REG_MH_MMU_TRAN_ERROR,
				     mmu->dummyspace.gpuaddr);

		kgsl_yamato_tlbinvalidate(device);

		mmu->flags |= KGSL_FLAGS_STARTED;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int
kgsl_mmu_map(struct kgsl_pagetable *pagetable,
				unsigned int physaddr,
				int range,
				unsigned int protflags,
				unsigned int *gpuaddr)
{
	int numpages;
	unsigned int pte, superpte, ptefirst, ptelast;
	int flushtlb;
	struct kgsl_mmu *mmu = NULL;

	KGSL_MEM_VDBG("enter (pt=%p, physaddr=%08x, range=%08d, gpuaddr=%p)\n",
		      pagetable, physaddr, range, gpuaddr);

	if (!kgsl_mmu_enable) {
		KGSL_MEM_VDBG("mmu disabled\n");
		*gpuaddr = physaddr;
		return 0;
	}
	mmu = pagetable->mmu;

	BUG_ON(mmu == NULL);
	BUG_ON(protflags & ~(GSL_PT_PAGE_RV | GSL_PT_PAGE_WV));
	BUG_ON(protflags == 0);
	BUG_ON(range <= 0);

	*gpuaddr = gen_pool_alloc(pagetable->pool, range);
	if (*gpuaddr == 0) {
		KGSL_MEM_ERR("gen_pool_alloc failed\n");
		return -ENOMEM;
	}



	numpages = (range >> KGSL_PAGESIZE_SHIFT);
	if (range & (KGSL_PAGESIZE - 1))
		numpages++;

	ptefirst = kgsl_pt_entry_get(pagetable, *gpuaddr);
	ptelast = ptefirst + numpages;

	pte = ptefirst;
	flushtlb = 0;


	superpte = ptefirst & (GSL_PT_SUPER_PTE - 1);
	for (pte = superpte; pte < ptefirst; pte++) {
		/* tlb needs to be flushed only when a dirty superPTE
		   gets backed */
		if (kgsl_pt_map_isdirty(pagetable, pte)) {
			flushtlb = 1;
			break;
		}
	}

	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		uint32_t val = kgsl_pt_map_getaddr(pagetable, pte);
		BUG_ON(val != 0 && val != GSL_PT_PAGE_DIRTY);
#endif
		if (kgsl_pt_map_isdirty(pagetable, pte))
			flushtlb = 1;
		/* mark pte as in use */
		kgsl_pt_map_set(pagetable, pte, physaddr | protflags);
		physaddr += KGSL_PAGESIZE;
	}

	/* set superpte to end of next superpte */
	superpte = (ptelast + (GSL_PT_SUPER_PTE - 1))
			& (GSL_PT_SUPER_PTE - 1);
	for (pte = ptelast; pte < superpte; pte++) {
		/* tlb needs to be flushed only when a dirty superPTE
		   gets backed */
		if (kgsl_pt_map_isdirty(pagetable, pte)) {
			flushtlb = 1;
			break;
		}
	}
	KGSL_MEM_INFO("pt %p p %08x g %08x pte f %d l %d n %d f %d\n",
		      pagetable, physaddr, *gpuaddr, ptefirst, ptelast,
		      numpages, flushtlb);

	kgsl_yamato_tlbinvalidate(mmu->device);


	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int
kgsl_mmu_unmap(struct kgsl_pagetable *pagetable, unsigned int gpuaddr,
		int range)
{
	unsigned int numpages;
	unsigned int pte, ptefirst, ptelast;

	KGSL_MEM_VDBG("enter (pt=%p, gpuaddr=0x%08x, range=%d)\n",
			pagetable, gpuaddr, range);

	if (!kgsl_mmu_enable) {
		KGSL_MEM_VDBG("mmu disabled\n");
		return 0;
	}

	BUG_ON(range <= 0);

	numpages = (range >> KGSL_PAGESIZE_SHIFT);
	if (range & (KGSL_PAGESIZE - 1))
		numpages++;

	ptefirst = kgsl_pt_entry_get(pagetable, gpuaddr);
	ptelast = ptefirst + numpages;

	KGSL_MEM_INFO("pt %p gpu %08x pte first %d last %d numpages %d\n",
		      pagetable, gpuaddr, ptefirst, ptelast, numpages);

	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		BUG_ON(!kgsl_pt_map_getaddr(pagetable, pte));
#endif
		kgsl_pt_map_set(pagetable, pte, GSL_PT_PAGE_DIRTY);
	}

	gen_pool_free(pagetable->pool, gpuaddr, range);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int kgsl_mmu_close(struct kgsl_device *device)
{
	/*
	 *  close device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;
#ifdef _DEBUG
	int i;
#endif /* _DEBUG */

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		/* disable mh interrupts */
		KGSL_MEM_DBG("disabling mmu interrupts\n");
		kgsl_yamato_regwrite(device, REG_MH_INTERRUPT_MASK, 0);

		/* disable MMU */
		kgsl_yamato_regwrite(device, REG_MH_MMU_CONFIG, 0x00000000);

		if (mmu->dummyspace.gpuaddr)
			kgsl_sharedmem_free(&mmu->dummyspace);

		mmu->flags &= ~KGSL_FLAGS_STARTED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED0;

		if (mmu->defaultpagetable != NULL) {
			kgsl_mmu_destroypagetableobject(mmu->defaultpagetable);
			mmu->defaultpagetable = NULL;
		}
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
