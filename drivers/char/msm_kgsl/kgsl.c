/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/android_pmem.h>
#include <linux/pm_qos_params.h>

#include <linux/delay.h>
#include <asm/atomic.h>
#include <mach/internal_power_rail.h>

#include "kgsl.h"
#include "kgsl_drawctxt.h"
#include "kgsl_ringbuffer.h"
#include "kgsl_log.h"
#include "kgsl_drm.h"

struct kgsl_file_private {
	struct list_head list;
	struct list_head pmem_list;
	uint32_t ctxt_id_mask;
	struct kgsl_pagetable *pagetable;
};

/*this is used for logging, so that we can call the dev_printk
 functions without export struct kgsl_driver everywhere*/
struct device *kgsl_driver_getdevnode(void)
{
	BUG_ON(kgsl_driver.pdev == NULL);
	return &kgsl_driver.pdev->dev;
}

int kgsl_pwrctrl(unsigned int pwrflag)
{
	switch (pwrflag) {
	case KGSL_PWRFLAGS_CLK_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_CLK_ON) {
			if (kgsl_driver.grp_pclk)
				clk_disable(kgsl_driver.grp_pclk);

			clk_disable(kgsl_driver.grp_clk);

			clk_disable(kgsl_driver.imem_clk);
			pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ,
				DRIVER_NAME, PM_QOS_DEFAULT_VALUE);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_CLK_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_CLK_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_CLK_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_CLK_OFF) {
			if (kgsl_driver.max_axi_freq) {
				pm_qos_update_requirement(
					PM_QOS_SYSTEM_BUS_FREQ, DRIVER_NAME,
					kgsl_driver.max_axi_freq);
			}
			if (kgsl_driver.grp_pclk)
				clk_enable(kgsl_driver.grp_pclk);
			clk_enable(kgsl_driver.grp_clk);
			clk_enable(kgsl_driver.imem_clk);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_CLK_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_CLK_ON;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_POWER_ON) {
			internal_pwr_rail_ctl(PWR_RAIL_GRP_CLK, KGSL_FALSE);
			internal_pwr_rail_mode(PWR_RAIL_GRP_CLK,
					PWR_RAIL_CTL_AUTO);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_POWER_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_POWER_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_POWER_OFF) {
			internal_pwr_rail_mode(PWR_RAIL_GRP_CLK,
					PWR_RAIL_CTL_MANUAL);
			internal_pwr_rail_ctl(PWR_RAIL_GRP_CLK, KGSL_TRUE);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_POWER_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_POWER_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

/*Suspend function*/
static int kgsl_suspend(struct platform_device *dev, pm_message_t state)
{
	mutex_lock(&kgsl_driver.mutex);
	if (kgsl_driver.power_flags != 0) {
		if (kgsl_driver.yamato_device.hwaccess_blocked == KGSL_FALSE)
			kgsl_yamato_suspend(&kgsl_driver.yamato_device);
		kgsl_driver.is_suspended = KGSL_TRUE;
	}
	mutex_unlock(&kgsl_driver.mutex);
	return KGSL_SUCCESS;
}

/*Resume function*/
static int kgsl_resume(struct platform_device *dev)
{
	mutex_lock(&kgsl_driver.mutex);
	if (kgsl_driver.power_flags != 0) {
		kgsl_yamato_wake(&kgsl_driver.yamato_device);
		kgsl_driver.is_suspended = KGSL_FALSE;
	}
	mutex_unlock(&kgsl_driver.mutex);
	return KGSL_SUCCESS;
}

/* file operations */
static int kgsl_first_open_locked(void)
{
	int result = 0;

	BUG_ON(kgsl_driver.grp_clk == NULL);
	BUG_ON(kgsl_driver.imem_clk == NULL);

	kgsl_driver.power_flags =
	    KGSL_PWRFLAGS_CLK_OFF | KGSL_PWRFLAGS_POWER_OFF;
	kgsl_pwrctrl(KGSL_PWRFLAGS_POWER_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_CLK_ON);
	kgsl_driver.is_suspended = KGSL_FALSE;

	/* init memory apertures */
	result = kgsl_sharedmem_init(&kgsl_driver.shmem);
	if (result != 0)
		goto done;

	/* init devices */
	result = kgsl_yamato_init(&kgsl_driver.yamato_device,
					&kgsl_driver.yamato_config);
	if (result != 0)
		goto done;

	result = kgsl_yamato_start(&kgsl_driver.yamato_device, 0);
	if (result != 0)
		goto done;

	enable_irq(kgsl_driver.interrupt_num);
done:
	return result;
}

static int kgsl_last_release_locked(void)
{
	BUG_ON(kgsl_driver.grp_clk == NULL);
	BUG_ON(kgsl_driver.imem_clk == NULL);

	disable_irq(kgsl_driver.interrupt_num);

	kgsl_yamato_stop(&kgsl_driver.yamato_device);

	/* close devices */
	kgsl_yamato_close(&kgsl_driver.yamato_device);

	/* shutdown memory apertures */
	kgsl_sharedmem_close(&kgsl_driver.shmem);

	kgsl_pwrctrl(KGSL_PWRFLAGS_CLK_OFF);
	kgsl_pwrctrl(KGSL_PWRFLAGS_POWER_OFF);
	kgsl_driver.power_flags = 0;

	return 0;
}

static int kgsl_release(struct inode *inodep, struct file *filep)
{
	int result = 0;
	unsigned int i;
	struct kgsl_pmem_entry *entry, *entry_tmp;
	struct kgsl_file_private *private = NULL;

	KGSL_PRE_HWACCESS();

	private = filep->private_data;
	BUG_ON(private == NULL);
	filep->private_data = NULL;
	list_del(&private->list);

	for (i = 0; i < KGSL_CONTEXT_MAX; i++)
		if (private->ctxt_id_mask & (1 << i))
			kgsl_drawctxt_destroy(&kgsl_driver.yamato_device, i);

	list_for_each_entry_safe(entry, entry_tmp, &private->pmem_list, list)
		kgsl_remove_pmem_entry(entry);

	if (private->pagetable != NULL) {
		kgsl_yamato_cleanup_pt(&kgsl_driver.yamato_device,
					private->pagetable);
		kgsl_mmu_destroypagetableobject(private->pagetable);
		private->pagetable = NULL;
	}

	kfree(private);

	if (atomic_dec_return(&kgsl_driver.open_count) == 0) {
		KGSL_DRV_VDBG("last_release\n");
		result = kgsl_last_release_locked();
	}

	KGSL_POST_HWACCESS();

	return result;
}

static int kgsl_open(struct inode *inodep, struct file *filep)
{
	int result = 0;
	struct kgsl_file_private *private = NULL;

	KGSL_DRV_DBG("file %p pid %d\n", filep, task_pid_nr(current));


	if (filep->f_flags & O_EXCL) {
		KGSL_DRV_ERR("O_EXCL not allowed\n");
		return -EBUSY;
	}

	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (private == NULL) {
		KGSL_DRV_ERR("cannot allocate file private data\n");
		return -ENOMEM;
	}

	mutex_lock(&kgsl_driver.mutex);

	private->ctxt_id_mask = 0;
	INIT_LIST_HEAD(&private->pmem_list);

	filep->private_data = private;

	list_add(&private->list, &kgsl_driver.client_list);

	if (atomic_inc_return(&kgsl_driver.open_count) == 1) {
		result = kgsl_first_open_locked();
		if (result != 0)
			goto done;
	}
	/*NOTE: this must happen after first_open */
	if (kgsl_mmu_enable) {
		private->pagetable =
			kgsl_mmu_createpagetableobject(
				&kgsl_driver.yamato_device.mmu);
		if (private->pagetable == NULL) {
			result = -ENOMEM;
			goto done;
		}
		result = kgsl_yamato_setup_pt(&kgsl_driver.yamato_device,
						private->pagetable);
		if (result)
			goto done;
	}

done:
	mutex_unlock(&kgsl_driver.mutex);
	if (result != 0)
		kgsl_release(inodep, filep);
	return result;
}


/*call with driver locked */
static struct kgsl_pmem_entry *
kgsl_sharedmem_find(struct kgsl_file_private *private, unsigned int gpuaddr)
{
	struct kgsl_pmem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->pmem_list, list) {
		if (entry->memdesc.gpuaddr == gpuaddr) {
			result = entry;
			break;
		}
	}
	return result;
}

/*call with driver locked */
struct kgsl_pmem_entry *
kgsl_sharedmem_find_region(struct kgsl_file_private *private,
				unsigned int gpuaddr,
				size_t size)
{
	struct kgsl_pmem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->pmem_list, list) {
		if (gpuaddr >= entry->memdesc.gpuaddr &&
		    ((gpuaddr + size) <=
			(entry->memdesc.gpuaddr + entry->memdesc.size))) {
			result = entry;
			break;
		}
	}

	return result;
}

/*call all ioctl sub functions with driver locked*/

static long kgsl_ioctl_device_getproperty(struct kgsl_file_private *private,
					 void __user *arg)
{
	int result = 0;
	struct kgsl_device_getproperty param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	result = kgsl_yamato_getproperty(&kgsl_driver.yamato_device,
					 param.type,
					 param.value, param.sizebytes);
done:
	return result;
}

static long kgsl_ioctl_device_regread(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_regread param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	result = kgsl_yamato_regread(&kgsl_driver.yamato_device,
				     param.offsetwords, &param.value);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}


static long kgsl_ioctl_device_waittimestamp(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_waittimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	mutex_unlock(&kgsl_driver.mutex);
	result = kgsl_yamato_waittimestamp(&kgsl_driver.yamato_device,
				     param.timestamp,
				     param.timeout);
	mutex_lock(&kgsl_driver.mutex);

	kgsl_yamato_runpending(&kgsl_driver.yamato_device);
done:
	return result;
}

static long kgsl_ioctl_rb_issueibcmds(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_ringbuffer_issueibcmds param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.drawctxt_id >= KGSL_CONTEXT_MAX
		|| (private->ctxt_id_mask & 1 << param.drawctxt_id) == 0) {
		result = -EINVAL;
		KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
			      param.drawctxt_id);
		result = -EINVAL;
		goto done;
	}

	if (kgsl_sharedmem_find_region(private, param.ibaddr,
				param.sizedwords*sizeof(uint32_t)) == NULL) {
		KGSL_DRV_ERR("invalid cmd buffer ibaddr %08x sizedwords %d\n",
			      param.ibaddr, param.sizedwords);
		result = -EINVAL;
		goto done;

	}

	result = kgsl_ringbuffer_issueibcmds(&kgsl_driver.yamato_device,
					     param.drawctxt_id,
					     param.ibaddr,
					     param.sizedwords,
					     &param.timestamp,
					     param.flags);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_rb_readtimestamp(struct kgsl_file_private *private,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_ringbuffer_readtimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	param.timestamp =
		kgsl_ringbuffer_readtimestamp(&kgsl_driver.yamato_device,
							param.type);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_rb_freememontimestamp(struct kgsl_file_private *private,
					    void __user *arg)
{
	int result = 0;
	struct kgsl_ringbuffer_freememontimestamp param;
	struct kgsl_pmem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.gpuaddr) {
		entry = kgsl_sharedmem_find(private, param.gpuaddr);
		if (entry == NULL) {
			KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
			result = -EINVAL;
			goto done;
		}

		result = kgsl_ringbuffer_freememontimestamp(
					&kgsl_driver.yamato_device,
					entry,
					param.timestamp,
					param.type);
	}

	kgsl_yamato_runpending(&kgsl_driver.yamato_device);

done:
	return result;
}

static long kgsl_ioctl_drawctxt_create(struct kgsl_file_private *private,
				      void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_create param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	result = kgsl_drawctxt_create(&kgsl_driver.yamato_device,
					private->pagetable,
					param.flags,
					&param.drawctxt_id);
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	private->ctxt_id_mask |= 1 << param.drawctxt_id;

done:
	return result;
}

static long kgsl_ioctl_drawctxt_destroy(struct kgsl_file_private *private,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_destroy param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.drawctxt_id >= KGSL_CONTEXT_MAX
		|| (private->ctxt_id_mask & 1 << param.drawctxt_id) == 0) {
		result = -EINVAL;
		goto done;
	}

	result = kgsl_drawctxt_destroy(&kgsl_driver.yamato_device,
					param.drawctxt_id);
	if (result == 0)
		private->ctxt_id_mask &= ~(1 << param.drawctxt_id);

done:
	return result;
}

void kgsl_remove_pmem_entry(struct kgsl_pmem_entry *entry)
{
	KGSL_DRV_DBG("unlocked pmem fd %p\n", entry->pmem_file);
	put_pmem_file(entry->pmem_file);

	/* remove the entry from list and free_list if it exists */
	if (entry->list.prev)
		list_del(&entry->list);
	if (entry->free_list.prev)
		list_del(&entry->free_list);

	kgsl_mmu_unmap(entry->memdesc.pagetable,
			entry->memdesc.gpuaddr,
			entry->memdesc.size);

	kfree(entry);

}

static long kgsl_ioctl_sharedmem_free(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_free param;
	struct kgsl_pmem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.gpuaddr) {
		entry = kgsl_sharedmem_find(private, param.gpuaddr);
		if (entry == NULL) {
			KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
			result = -EINVAL;
			goto done;
		}

		kgsl_remove_pmem_entry(entry);
	}
done:
	return result;
}

static long kgsl_ioctl_sharedmem_from_pmem(struct kgsl_file_private *private,
						void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_from_pmem param;
	struct kgsl_pmem_entry *entry = NULL;
	unsigned long start = 0, vstart = 0, len = 0;
	struct file *pmem_file = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	if (get_pmem_file(param.pmem_fd, &start, &vstart, &len, &pmem_file)) {
		result = -EINVAL;
		goto error;
	}
	KGSL_MEM_INFO("pmem file %p start 0x%lx vstart 0x%lx len 0x%lx\n",
			pmem_file, start, vstart, len);
	KGSL_DRV_DBG("locked pmem file %p\n", pmem_file);

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		result = -ENOMEM;
		goto error_put_pmem;
	}

	entry->pmem_file = pmem_file;
	list_add(&entry->list, &private->pmem_list);

	entry->memdesc.pagetable = private->pagetable;
	entry->memdesc.size = len;
	/*we shouldn't need to write here from kernel mode */
	entry->memdesc.hostptr = NULL;

	entry->memdesc.physaddr = start;
	if (kgsl_mmu_enable)  {
		result = kgsl_mmu_map(private->pagetable, start, len,
					GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
					&entry->memdesc.gpuaddr);
		if (result != 0)
			goto error_free_entry;
	} else {
		entry->memdesc.gpuaddr = start;
	}

	param.gpuaddr = entry->memdesc.gpuaddr;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_copy_to_user;
	}
	return 0;
error_copy_to_user:
	kgsl_mmu_unmap(entry->memdesc.pagetable,
			entry->memdesc.gpuaddr,
			entry->memdesc.size);

error_free_entry:
	kfree(entry);

error_put_pmem:
	KGSL_DRV_DBG("unlocked pmem file %p\n", pmem_file);
	put_pmem_file(pmem_file);

error:
	return result;
}


static long kgsl_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result = 0;
	struct kgsl_file_private *private = filep->private_data;

	BUG_ON(private == NULL);

	KGSL_DRV_VDBG("filep %p cmd 0x%08x arg 0x%08lx\n", filep, cmd, arg);
	KGSL_PRE_HWACCESS();
	switch (cmd) {

	case IOCTL_KGSL_DEVICE_GETPROPERTY:
		result =
		    kgsl_ioctl_device_getproperty(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_REGREAD:
		result = kgsl_ioctl_device_regread(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_WAITTIMESTAMP:
		result = kgsl_ioctl_device_waittimestamp(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS:
		result = kgsl_ioctl_rb_issueibcmds(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_RINGBUFFER_READTIMESTAMP:
		result =
		    kgsl_ioctl_rb_readtimestamp(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_RINGBUFFER_FREEMEMONTIMESTAMP:
		result =
		    kgsl_ioctl_rb_freememontimestamp(private,
						    (void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_CREATE:
		result = kgsl_ioctl_drawctxt_create(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_DESTROY:
		result =
		    kgsl_ioctl_drawctxt_destroy(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FREE:
		result = kgsl_ioctl_sharedmem_free(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FROM_PMEM:
		result = kgsl_ioctl_sharedmem_from_pmem(private,
							(void __user *)arg);
		break;

	default:
		KGSL_DRV_ERR("invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	KGSL_POST_HWACCESS();
	KGSL_DRV_VDBG("result %d\n", result);
	return result;
}

static int kgsl_mmap(struct file *file, struct vm_area_struct *vma)
{
	int result;
	struct kgsl_memdesc *memdesc = NULL;
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	unsigned long vma_offset = vma->vm_pgoff << PAGE_SHIFT;
	struct kgsl_device *device = NULL;

	mutex_lock(&kgsl_driver.mutex);

	device = &kgsl_driver.yamato_device;

	/*allow yamato memstore to be mapped read only */
	if (vma_offset == device->memstore.physaddr) {
		if (vma->vm_flags & VM_WRITE) {
			result = -EPERM;
			goto done;
		}
		memdesc = &device->memstore;
	}

	if (memdesc->size != vma_size) {
		KGSL_MEM_ERR("file %p bad size %ld, should be %d\n",
			file, vma_size, memdesc->size);
		result = -EINVAL;
		goto done;
	}
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	result = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma_size, vma->vm_page_prot);
	if (result != 0) {
		KGSL_MEM_ERR("remap_pfn_range returned %d\n",
				result);
		goto done;
	}
done:
	mutex_unlock(&kgsl_driver.mutex);
	return result;
}

static const struct file_operations kgsl_fops = {
	.owner = THIS_MODULE,
	.release = kgsl_release,
	.open = kgsl_open,
	.mmap = kgsl_mmap,
	.unlocked_ioctl = kgsl_ioctl,
};


struct kgsl_driver kgsl_driver = {
	.misc = {
		 .name = DRIVER_NAME,
		 .minor = MISC_DYNAMIC_MINOR,
		 .fops = &kgsl_fops,
	 },
	.open_count = ATOMIC_INIT(0),
	.mutex = __MUTEX_INITIALIZER(kgsl_driver.mutex),
};


static void kgsl_driver_cleanup(void)
{

	if (kgsl_driver.interrupt_num > 0) {
		if (kgsl_driver.have_irq) {
			free_irq(kgsl_driver.interrupt_num, NULL);
			kgsl_driver.have_irq = 0;
		}
		kgsl_driver.interrupt_num = 0;
	}

	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, DRIVER_NAME);

	if (kgsl_driver.grp_pclk) {
		clk_put(kgsl_driver.grp_pclk);
		kgsl_driver.grp_pclk = NULL;
	}

	if (kgsl_driver.grp_clk) {
		clk_put(kgsl_driver.grp_clk);
		kgsl_driver.grp_clk = NULL;
	}

	if (kgsl_driver.imem_clk != NULL) {
		clk_put(kgsl_driver.imem_clk);
		kgsl_driver.imem_clk = NULL;
	}

	kgsl_driver.pdev = NULL;

}


static int __devinit kgsl_platform_probe(struct platform_device *pdev)
{
	int result = 0;
	struct clk *clk;
	struct resource *res = NULL;
	struct kgsl_platform_data *pdata = NULL;

	kgsl_debug_init();

	INIT_LIST_HEAD(&kgsl_driver.client_list);
	/*acquire clocks */
	BUG_ON(kgsl_driver.grp_clk != NULL);
	BUG_ON(kgsl_driver.imem_clk != NULL);

	kgsl_driver.pdev = pdev;

	clk = clk_get(&pdev->dev, "grp_pclk");
	if (IS_ERR(clk))
		clk = NULL;
	kgsl_driver.grp_pclk = clk;

	clk = clk_get(&pdev->dev, "grp_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(grp_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.grp_clk = clk;

	clk = clk_get(&pdev->dev, "imem_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(imem_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.imem_clk = clk;
	kgsl_driver.power_flags = 0;

	pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ, DRIVER_NAME,
				PM_QOS_DEFAULT_VALUE);

	pdata = pdev->dev.platform_data;
	if (pdata)
		kgsl_driver.max_axi_freq = pdata->max_axi_freq;

	/*acquire interrupt */
	kgsl_driver.interrupt_num = platform_get_irq(pdev, 0);
	if (kgsl_driver.interrupt_num <= 0) {
		KGSL_DRV_ERR("platform_get_irq() returned %d\n",
			       kgsl_driver.interrupt_num);
		result = -EINVAL;
		goto done;
	}
	result = request_irq(kgsl_driver.interrupt_num, kgsl_yamato_isr,
				IRQF_TRIGGER_HIGH, DRIVER_NAME, NULL);
	if (result) {
		KGSL_DRV_ERR("request_irq(%d) returned %d\n",
			      kgsl_driver.interrupt_num, result);
		goto done;
	}
	kgsl_driver.have_irq = 1;
	disable_irq(kgsl_driver.interrupt_num);

	result = kgsl_yamato_config(&kgsl_driver.yamato_config, pdev);
	if (result != 0)
		goto done;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "kgsl_phys_memory");
	if (res == NULL) {
		result = -EINVAL;
		goto done;
	}

	kgsl_driver.shmem.physbase = res->start;
	kgsl_driver.shmem.size = resource_size(res);
	result = kgsl_drm_init(pdev);

done:
	if (result)
		kgsl_driver_cleanup();
	else
		result = misc_register(&kgsl_driver.misc);

	return result;
}

static int kgsl_platform_remove(struct platform_device *pdev)
{

	kgsl_driver_cleanup();
	kgsl_drm_exit();
	misc_deregister(&kgsl_driver.misc);

	return 0;
}

static struct platform_driver kgsl_platform_driver = {
	.probe = kgsl_platform_probe,
	.remove = __devexit_p(kgsl_platform_remove),
	.suspend = kgsl_suspend,
	.resume = kgsl_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME
	}
};

static int __init kgsl_mod_init(void)
{
	return platform_driver_register(&kgsl_platform_driver);
}

static void __exit kgsl_mod_exit(void)
{
	platform_driver_unregister(&kgsl_platform_driver);
}

#ifdef MODULE
module_init(kgsl_mod_init);
#else
late_initcall(kgsl_mod_init);
#endif
module_exit(kgsl_mod_exit);

MODULE_DESCRIPTION("3D graphics driver for QSD8x50 and MSM7x27");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:kgsl");
