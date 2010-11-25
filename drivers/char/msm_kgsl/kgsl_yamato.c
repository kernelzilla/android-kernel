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
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>


#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"

#include "yamato_reg.h"

#define GSL_RBBM_INT_MASK \
	 (RBBM_INT_CNTL__RDERR_INT_MASK |  \
	  RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK)

#define GSL_SQ_INT_MASK \
	(SQ_INT_CNTL__PS_WATCHDOG_MASK | \
	 SQ_INT_CNTL__VS_WATCHDOG_MASK)

/* Yamato MH arbiter config*/
#define KGSL_CFG_YAMATO_MHARB \
	(0x10 \
		| (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
		| (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

static int kgsl_yamato_gmeminit(struct kgsl_device *device)
{
	unsigned int rb_edram_info =  0;
	unsigned int gmem_size;
	unsigned int edram_value = 0;

	/* make sure edram range is aligned to size */
	BUG_ON(device->gmemspace.gpu_base & (device->gmemspace.sizebytes - 1));

	/* get edram_size value equivalent */
	gmem_size = (device->gmemspace.sizebytes >> 14);
	while (gmem_size >>= 1)
		edram_value++;

	rb_edram_info = (edram_value & RB_EDRAM_INFO__EDRAM_SIZE_MASK)
			| (0 << RB_EDRAM_INFO__EDRAM_MAPPING_MODE__SHIFT)
			| (((device->gmemspace.gpu_base >> 14)
				& RB_EDRAM_INFO__EDRAM_RANGE_MASK)
				<< RB_EDRAM_INFO__EDRAM_RANGE__SHIFT);

	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, rb_edram_info);

	return 0;
}

static int kgsl_yamato_gmemclose(struct kgsl_device *device)
{
	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, 0x00000000);

	return 0;
}

void kgsl_yamato_rbbm_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int rderr = 0;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_RBBM_INT_STATUS, &status);

	if (status & RBBM_INT_CNTL__RDERR_INT_MASK) {
		kgsl_yamato_regread(device, REG_RBBM_READ_ERROR, &rderr);
		KGSL_DRV_FATAL("rbbm read error interrupt: %08x\n", rderr);
	} else if (status & RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK) {
		KGSL_DRV_DBG("rbbm display update interrupt\n");
	} else if (status & RBBM_INT_CNTL__GUI_IDLE_INT_MASK) {
		KGSL_DRV_DBG("rbbm gui idle interrupt\n");
	} else {
		KGSL_CMD_DBG("bad bits in REG_CP_INT_STATUS %08x\n", status);
	}

	status &= GSL_RBBM_INT_MASK;
	kgsl_yamato_regwrite(device, REG_RBBM_INT_ACK, status);

	KGSL_DRV_VDBG("return\n");
}

void kgsl_yamato_sq_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_SQ_INT_STATUS, &status);

	if (status & SQ_INT_CNTL__PS_WATCHDOG_MASK)
		KGSL_DRV_DBG("sq ps watchdog interrupt\n");
	else if (status & SQ_INT_CNTL__VS_WATCHDOG_MASK)
		KGSL_DRV_DBG("sq vs watchdog interrupt\n");
	else
		KGSL_DRV_DBG("bad bits in REG_SQ_INT_STATUS %08x\n", status);


	status &= GSL_SQ_INT_MASK;
	kgsl_yamato_regwrite(device, REG_SQ_INT_ACK, status);

	KGSL_DRV_VDBG("return\n");
}

irqreturn_t kgsl_yamato_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;

	struct kgsl_device *device = &kgsl_driver.yamato_device;
	unsigned int status;

	kgsl_yamato_regread(device, REG_MASTER_INT_SIGNAL, &status);

	if (status & MASTER_INT_SIGNAL__MH_INT_STAT) {
		kgsl_mh_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__CP_INT_STAT) {
		kgsl_cp_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__RBBM_INT_STAT) {
		kgsl_yamato_rbbm_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__SQ_INT_STAT) {
		kgsl_yamato_sq_intrcallback(device);
		result = IRQ_HANDLED;
	}


	return result;
}

int kgsl_yamato_tlbinvalidate(struct kgsl_device *device)
{
	unsigned int link[4];
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

	if (!kgsl_mmu_enable)
		return 0;


	KGSL_MEM_DBG("device %p ctxt %p pt %p\n",
			device,
			device->drawctxt_active,
			device->mmu.hwpagetable);
	/* if there is an active draw context, invalidate via command stream,
	* otherwise invalidate via direct register writes
	*/
	if (device->drawctxt_active) {
		/* wait for graphics pipe to be idle */
		link[0] = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
		link[1] = 0x00000000;

		link[2] = pm4_type0_packet(REG_MH_MMU_INVALIDATE, 1);
		link[3] = mh_mmu_invalidate;

		KGSL_MEM_DBG("cmds\n");
		kgsl_ringbuffer_issuecmds(device, device->drawctxt_active, 1,
					  &link[0], 4,
					  KGSL_CONTEXT_SAVE_GMEM);
	} else {
		KGSL_MEM_DBG("regs\n");
		kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

		kgsl_yamato_regwrite(device, REG_MH_MMU_INVALIDATE,
				     mh_mmu_invalidate);
	}

	return 0;
}
/*in order to have all memory be virtualized, we need to have some "global"
* memory regions at the same virtual addresses. This function sets it up.
* It should be called first for the mmu->defaultpagetable and then for
* every pagetable created for clients. Note that the first time through, it
* sets the gpuaddr field in all memdescs.
* Also, there's probably a less kludgy way to do this.
*/
int kgsl_yamato_setup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int gpuaddr = 0;
	BUG_ON(device->ringbuffer.buffer_desc.physaddr == 0);
	BUG_ON(device->ringbuffer.memptrs_desc.physaddr == 0);
	BUG_ON(device->memstore.physaddr == 0);

	result = kgsl_mmu_map(pagetable,
				device->ringbuffer.buffer_desc.physaddr,
				device->ringbuffer.buffer_desc.size,
				GSL_PT_PAGE_RV,
				&gpuaddr);

	if (device->ringbuffer.buffer_desc.gpuaddr == 0)
		device->ringbuffer.buffer_desc.gpuaddr = gpuaddr;
	BUG_ON(device->ringbuffer.buffer_desc.gpuaddr != gpuaddr);

	result = kgsl_mmu_map(pagetable,
				device->ringbuffer.memptrs_desc.physaddr,
				device->ringbuffer.memptrs_desc.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
				&gpuaddr);

	if (device->ringbuffer.memptrs_desc.gpuaddr == 0)
		device->ringbuffer.memptrs_desc.gpuaddr = gpuaddr;
	BUG_ON(device->ringbuffer.memptrs_desc.gpuaddr != gpuaddr);

	result = kgsl_mmu_map(pagetable,
				device->memstore.physaddr,
				device->memstore.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
				&gpuaddr);

	if (device->memstore.gpuaddr == 0)
		device->memstore.gpuaddr = gpuaddr;
	BUG_ON(device->memstore.gpuaddr != gpuaddr);

	return result;
}

int kgsl_yamato_cleanup_pt(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable)
{
	int result = 0;
	BUG_ON(device->ringbuffer.buffer_desc.gpuaddr == 0);
	BUG_ON(device->ringbuffer.memptrs_desc.gpuaddr == 0);
	BUG_ON(device->memstore.gpuaddr == 0);

	result = kgsl_mmu_unmap(pagetable,
				device->ringbuffer.buffer_desc.gpuaddr,
				device->ringbuffer.buffer_desc.size);

	result = kgsl_mmu_unmap(pagetable,
				device->ringbuffer.memptrs_desc.gpuaddr,
				device->ringbuffer.memptrs_desc.size);

	result = kgsl_mmu_unmap(pagetable,
				device->memstore.gpuaddr,
				device->memstore.size);
	return result;
}

int kgsl_yamato_setpagetable(struct kgsl_device *device)
{
	unsigned int link[10];
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

	if (!kgsl_mmu_enable)
		return 0;

	KGSL_MEM_DBG("device %p ctxt %p pt %p\n",
			device,
			device->drawctxt_active,
			device->mmu.hwpagetable);
	/* if there is an active draw context, set via command stream,
	* otherwise set via direct register writes
	*/
	if (device->drawctxt_active) {
		KGSL_MEM_DBG("cmds\n");
		/* wait for graphics pipe to be idle */
		link[0] = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
		link[1] = 0x00000000;

		/* set page table base */
		link[2] = pm4_type0_packet(REG_MH_MMU_PT_BASE, 1);
		link[3] = device->mmu.hwpagetable->base.gpuaddr;

		/* define virtual address range */
		link[4] = pm4_type0_packet(REG_MH_MMU_VA_RANGE, 1);
		link[5] =
		    (device->mmu.hwpagetable->
		     va_base | (device->mmu.hwpagetable->va_range >> 16));

		link[6] = pm4_type0_packet(REG_MH_MMU_INVALIDATE, 1);
		link[7] = mh_mmu_invalidate;

		link[8] = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
		link[9] = 0x00000000;

		kgsl_ringbuffer_issuecmds(device, device->drawctxt_active, 1,
					  &link[0], 10,
					  KGSL_CONTEXT_SAVE_GMEM);
	} else {
		KGSL_MEM_DBG("regs\n");
		kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

		kgsl_yamato_regwrite(device, REG_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		kgsl_yamato_regwrite(device, REG_MH_MMU_VA_RANGE,
				     (device->mmu.hwpagetable->
				      va_base | (device->mmu.hwpagetable->
						 va_range >> 16)));
		kgsl_yamato_regwrite(device, REG_MH_MMU_INVALIDATE,
				     mh_mmu_invalidate);
	}

	return 0;
}

static unsigned int
kgsl_yamato_getchipid(struct kgsl_device *device)
{
	unsigned int chipid;
	unsigned int coreid, majorid, minorid, patchid, revid;

	/* YDX */
	kgsl_yamato_regread(device, REG_RBBM_PERIPHID1, &coreid);
	coreid &= 0xF;

	kgsl_yamato_regread(device, REG_RBBM_PERIPHID2, &majorid);
	majorid = (majorid >> 4) & 0xF;

	kgsl_yamato_regread(device, REG_RBBM_PATCH_RELEASE, &revid);
	/* this is a 16bit field, but extremely unlikely it would ever get
	* this high
	*/
	minorid = ((revid >> 0)  & 0xFF);


	patchid = ((revid >> 16) & 0xFF);

	chipid  = ((coreid << 24) | (majorid << 16) |
			(minorid << 8) | (patchid << 0));

	/* Hardware revision 211 (8650) returns the wrong chip ID */
	if (chipid == KGSL_CHIPID_YAMATODX_REV21)
		chipid = KGSL_CHIPID_YAMATODX_REV211;

	return chipid;
}

int kgsl_yamato_init(struct kgsl_device *device, struct kgsl_devconfig *config)
{
	int status = -EINVAL;
	struct kgsl_memregion *regspace = &device->regspace;
	unsigned int memflags = KGSL_MEMFLAGS_ALIGNPAGE | KGSL_MEMFLAGS_CONPHYS;

	KGSL_DRV_VDBG("enter (device=%p, config=%p)\n", device, config);

	if (device->flags & KGSL_FLAGS_INITIALIZED) {
		KGSL_DRV_VDBG("return %d\n", 0);
		return 0;
	}
	memset(device, 0, sizeof(*device));

	init_waitqueue_head(&device->ib1_wq);

	memcpy(regspace, &config->regspace, sizeof(device->regspace));
	if (regspace->mmio_phys_base == 0 || regspace->sizebytes == 0) {
		KGSL_DRV_ERR("dev %d invalid regspace\n", device->id);
		goto error;
	}
	if (!request_mem_region(regspace->mmio_phys_base,
				regspace->sizebytes, DRIVER_NAME)) {
		KGSL_DRV_ERR("request_mem_region failed for register memory\n");
		status = -ENODEV;
		goto error;
	}

	regspace->mmio_virt_base = ioremap(regspace->mmio_phys_base,
					   regspace->sizebytes);
	KGSL_MEM_INFO("ioremap(regs) = %p\n", regspace->mmio_virt_base);
	if (regspace->mmio_virt_base == NULL) {
		KGSL_DRV_ERR("ioremap failed for register memory\n");
		status = -ENODEV;
		goto error_release_mem;
	}

	KGSL_DRV_INFO("dev %d regs phys 0x%08x size 0x%08x virt %p\n",
			device->id, regspace->mmio_phys_base,
			regspace->sizebytes, regspace->mmio_virt_base);


	memcpy(&device->gmemspace, &config->gmemspace,
			sizeof(device->gmemspace));

	device->id = KGSL_DEVICE_YAMATO;

	if (config->mmu_config) {
		device->mmu.config    = config->mmu_config;
		device->mmu.mpu_base  = config->mpu_base;
		device->mmu.mpu_range = config->mpu_range;
		device->mmu.va_base	  = config->va_base;
		device->mmu.va_range  = config->va_range;
	}

	device->chip_id = kgsl_yamato_getchipid(device);

	/*We need to make sure all blocks are powered up and clocked before
	*issuing a soft reset.  The overrides will be turned off (set to 0)
	*later in kgsl_yamato_start.
	*/
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0xfffffffe);
	if (device->chip_id == CHIP_REV_251)
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x000000ff);
	else
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0xffffffff);

	kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0xFFFFFFFF);

	/* The core is in an indeterminate state until the reset completes
	 * after 50ms.
	 */
	msleep(50);

	kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0x00000000);

	kgsl_yamato_regwrite(device, REG_RBBM_CNTL, 0x00004442);

	kgsl_yamato_regwrite(device, REG_MH_ARBITER_CONFIG,
				KGSL_CFG_YAMATO_MHARB);

	kgsl_yamato_regwrite(device, REG_SQ_VS_PROGRAM, 0x00000000);
	kgsl_yamato_regwrite(device, REG_SQ_PS_PROGRAM, 0x00000000);


	status = kgsl_mmu_init(device);
	if (status != 0) {
		status = -ENODEV;
		goto error_iounmap;
	}

	status = kgsl_sharedmem_alloc(memflags, sizeof(device->memstore),
					&device->memstore);
	if (status != 0)  {
		status = -ENODEV;
		goto error_close_mmu;
	}
	kgsl_sharedmem_set(&device->memstore, 0, 0, device->memstore.size);

	kgsl_yamato_regwrite(device, REG_RBBM_DEBUG, 0x00080000);

	device->flags |= KGSL_FLAGS_INITIALIZED;
	return 0;

error_iounmap:
	iounmap(regspace->mmio_virt_base);
	regspace->mmio_virt_base = NULL;
error_release_mem:
	release_mem_region(regspace->mmio_phys_base, regspace->sizebytes);
error_close_mmu:
	kgsl_mmu_close(device);
error:
	return status;
}

int kgsl_yamato_close(struct kgsl_device *device)
{
	struct kgsl_memregion *regspace = &device->regspace;

	if (device->memstore.hostptr)
		kgsl_sharedmem_free(&device->memstore);

	kgsl_mmu_close(device);

	if (regspace->mmio_virt_base != NULL) {
		KGSL_MEM_INFO("iounmap(regs) = %p\n", regspace->mmio_virt_base);
		iounmap(regspace->mmio_virt_base);
		regspace->mmio_virt_base = NULL;
		release_mem_region(regspace->mmio_phys_base,
					regspace->sizebytes);
	}

	KGSL_DRV_VDBG("return %d\n", 0);
	device->flags &= ~KGSL_FLAGS_INITIALIZED;
	return 0;
}

int kgsl_yamato_start(struct kgsl_device *device, uint32_t flags)
{
	int status = -EINVAL;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	if (!(device->flags & KGSL_FLAGS_INITIALIZED)) {
		KGSL_DRV_ERR("Trying to start uninitialized device.\n");
		return -EINVAL;
	}

	device->refcnt++;

	if (device->flags & KGSL_FLAGS_STARTED) {
		KGSL_DRV_VDBG("already started");
		return 0;
	}

	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0);
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0);

	KGSL_DRV_DBG("enabling RBBM interrupts mask 0x%08lx\n",
		     GSL_RBBM_INT_MASK);
	kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, GSL_RBBM_INT_MASK);

	/* make sure SQ interrupts are disabled */
	kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

	kgsl_yamato_gmeminit(device);

	status = kgsl_ringbuffer_init(device);
	if (status != 0) {
		kgsl_yamato_stop(device);
		return status;
	}

	status = kgsl_drawctxt_init(device);
	if (status != 0) {
		kgsl_yamato_stop(device);
		return status;
	}

	device->flags |= KGSL_FLAGS_STARTED;

	KGSL_DRV_VDBG("return %d\n", status);
	return status;
}

int kgsl_yamato_stop(struct kgsl_device *device)
{
	if (device->flags & KGSL_FLAGS_STARTED) {

		kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, 0);

		kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

		kgsl_drawctxt_close(device);

		kgsl_ringbuffer_close(&device->ringbuffer);

		kgsl_yamato_gmemclose(device);

		device->flags &= ~KGSL_FLAGS_STARTED;
	}

	return 0;
}

int kgsl_yamato_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
		{
			struct kgsl_devinfo devinfo;

			if (sizebytes != sizeof(devinfo)) {
				status = -EINVAL;
				break;
			}

			memset(&devinfo, 0, sizeof(devinfo));
			devinfo.device_id = device->id;
			devinfo.chip_id = device->chip_id;
			devinfo.mmu_enabled = kgsl_mmu_isenabled(&device->mmu);
			devinfo.gmem_hostbaseaddr =
				(unsigned int)device->gmemspace.mmio_virt_base;
			devinfo.gmem_gpubaseaddr = device->gmemspace.gpu_base;
			devinfo.gmem_sizebytes = device->gmemspace.sizebytes;

			if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
					0) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_DEVICE_SHADOW:
		{
			struct kgsl_shadowprop shadowprop;

			if (sizebytes != sizeof(shadowprop)) {
				status = -EINVAL;
				break;
			}
			memset(&shadowprop, 0, sizeof(shadowprop));
			if (device->memstore.hostptr) {
				/*NOTE: with mmu enabled, gpuaddr doesn't mean
				 * anything to mmap().
				 */
				shadowprop.gpuaddr = device->memstore.physaddr;
				shadowprop.size = device->memstore.size;
				shadowprop.flags = KGSL_FLAGS_INITIALIZED;
			}
			if (copy_to_user(value, &shadowprop,
				sizeof(shadowprop))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	default:
		status = -EINVAL;
	}

	return status;
}

int kgsl_yamato_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = -EINVAL;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;
	struct kgsl_mmu_debug mmu_dbg;
	unsigned int rbbm_status;
	int idle_count = 0;
#define IDLE_COUNT_MAX 1000000

	KGSL_DRV_VDBG("enter (device=%p, timeout=%d)\n", device, timeout);

	(void)timeout;

	/* first, wait until the CP has consumed all the commands in
	 * the ring buffer
	 */
	if (rb->flags & KGSL_FLAGS_STARTED) {
		do {
			idle_count++;
			GSL_RB_GET_READPTR(rb, &rb->rptr);

		} while (rb->rptr != rb->wptr && idle_count < IDLE_COUNT_MAX);
		if (idle_count == IDLE_COUNT_MAX) {
			KGSL_DRV_ERR("spun too long waiting for RB to idle\n");
			status = -EINVAL;
			kgsl_ringbuffer_dump(rb);
			kgsl_mmu_debug(&device->mmu, &mmu_dbg);
			goto done;
		}
	}
	/* now, wait for the GPU to finish its operations */
	for (idle_count = 0; idle_count < IDLE_COUNT_MAX; idle_count++) {
		kgsl_yamato_regread(device, REG_RBBM_STATUS, &rbbm_status);

		if (!(rbbm_status & RBBM_STATUS__GUI_ACTIVE_MASK)) {
			status = 0;
			break;
		}
	}

	if (idle_count == IDLE_COUNT_MAX) {
		KGSL_DRV_ERR("spun too long waiting for RBBM status to idle\n");
		status = -EINVAL;
		kgsl_ringbuffer_dump(rb);
		kgsl_mmu_debug(&device->mmu, &mmu_dbg);
		goto done;
	}
done:
	KGSL_DRV_VDBG("return %d\n", status);

	return status;
}

int kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;

	if (offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes) {
		KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
		return -ERANGE;
	}

	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	*value = readl(reg);

	return 0;
}

int kgsl_yamato_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	unsigned int *reg;

	if (offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes) {
		KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
		return -ERANGE;
	}

	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	writel(value, reg);

	return 0;
}

int kgsl_yamato_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	long timeout = 0;

	KGSL_DRV_INFO("enter (device=%p,timestamp=%d,timeout=0x%08x)\n",
			device, timestamp, msecs);

	timeout = wait_event_interruptible_timeout(device->ib1_wq,
			kgsl_ringbuffer_check_timestamp(device, timestamp),
			msecs_to_jiffies(msecs));

	if (timeout > 0)
		status = 0;
	else if (timeout == 0)
		status = -ETIMEDOUT;
	else
		status = timeout;

	KGSL_DRV_INFO("return %d\n", status);
	return status;
}

int kgsl_yamato_runpending(struct kgsl_device *device)
{
	if (device->flags & KGSL_FLAGS_INITIALIZED)
		kgsl_ringbuffer_memqueue_drain(device);
	return 0;
}

int __init kgsl_yamato_config(struct kgsl_devconfig *devconfig,
				struct platform_device *pdev)
{
	int result = 0;
	struct resource *res = NULL;

	memset(devconfig, 0, sizeof(*devconfig));

	/*find memory regions */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"kgsl_reg_memory");
	if (res == NULL) {
		KGSL_DRV_ERR("platform_get_resource_byname failed\n");
		result = -EINVAL;
		goto done;
	}
	KGSL_DRV_DBG("registers at %08x to %08x\n", res->start, res->end);
	devconfig->regspace.mmio_phys_base = res->start;
	devconfig->regspace.sizebytes = resource_size(res);

	devconfig->gmemspace.gpu_base = 0;
	devconfig->gmemspace.sizebytes = SZ_256K;

	/*note: for all of these behavior masks:
	 *	0 = do not translate
	 *	1 = translate within va_range, otherwise use phyisical
	 *	2 = translate within va_range, otherwise fault
	 */
	devconfig->mmu_config = 1 /* mmu enable */
		    | (2 << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)
		    | (2 << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT);

	/*TODO: these should probably be configurable from platform device
	 * stuff */
	devconfig->va_base = 0x66000000;
	devconfig->va_range = SZ_32M;

	/* turn off memory protection unit by setting acceptable physical
	 * address range to include all pages. Apparrently MPU causing
	 * problems.
	 */
	devconfig->mpu_base = 0x00000000;
	devconfig->mpu_range = 0xFFFFF000;

	result = 0;
done:
	return result;
}
