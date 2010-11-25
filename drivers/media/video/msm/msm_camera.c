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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/board.h>

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include <media/msm_camera.h>
#include <mach/camera.h>

#define MSM_MAX_CAMERA_SENSORS 5
#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
                                __func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
struct msm_device_t **msm_camera;
#endif

static struct class *msm_class;
static dev_t msm_devno;

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
static LIST_HEAD(msm_sensors);
struct  msm_control_device *g_v4l2_control_device;
int g_v4l2_opencnt;

#define __CONTAINS(r, v, l, field) ({                          \
	typeof(r) __r = r;                                      \
	typeof(v) __v = v;                                      \
	typeof(v) __e = __v + l;                                \
	int res = __v >= __r->field &&                          \
		__e <= __r->field + __r->len;                   \
	res;                                                    \
})

#define CONTAINS(r1, r2, field) ({                             \
	typeof(r2) __r2 = r2;                                   \
	 __CONTAINS(r1, __r2->field, __r2->len, field);          \
})	
#define IN_RANGE(r, v, field) ({                               \
	typeof(r) __r = r;                                      \
	typeof(v) __vv = v;                                     \
	int res = ((__vv >= __r->field) &&                      \
	(__vv < (__r->field + __r->len)));              \
res;                                                    \
})

#define OVERLAPS(r1, r2, field) ({                             \
	typeof(r1) __r1 = r1;                                   \
	typeof(r2) __r2 = r2;                                   \
	typeof(__r2->field) __v = __r2->field;                  \
	typeof(__v) __e = __v + __r2->len - 1;                  \
	int res = (IN_RANGE(__r1, __v, field) ||                \
		IN_RANGE(__r1, __e, field));                 \
	res;                                                    \
})

#define MSM_DRAIN_QUEUE_NOSYNC(sync, name) do {                        \
	struct msm_queue_cmd *qcmd = NULL;                      \
	CDBG("%s: draining queue "#name"\n", __func__);         \
	while (!list_empty(&(sync)->name)) {                    \
		qcmd = list_first_entry(&(sync)->name,          \
			struct msm_queue_cmd, list);            \
			list_del_init(&qcmd->list);                     \
			kfree(qcmd);                                    \
	};                                                      \
} while (0)

#define MSM_DRAIN_QUEUE(sync, name) do {			\
	unsigned long flags;					\
	spin_lock_irqsave(&(sync)->name##_lock, flags);		\
	MSM_DRAIN_QUEUE_NOSYNC(sync, name);			\
	spin_unlock_irqrestore(&(sync)->name##_lock, flags);	\
} while (0)

static int check_overlap(struct hlist_head *ptype,
			unsigned long paddr,
			unsigned long len)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region t = { .paddr = paddr, .len = len };
	struct hlist_node *node;

	hlist_for_each_entry(region, node, ptype, list) {
	if (CONTAINS(region, &t, paddr) ||
		CONTAINS(&t, region, paddr) ||
		OVERLAPS(region, &t, paddr)) {
		printk(KERN_ERR
			" region (PHYS %p len %ld)"
			" clashes with registered region"
			" (paddr %p len %ld)\n",
			(void *)t.paddr, t.len,
			(void *)region->paddr, region->len);
			return -1;
		}
	}

	return 0;
}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
uint8_t disable_lens_move = 0;
#endif

#if defined(CONFIG_MACH_PITTSBURGH)
static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info, struct file *file, unsigned long paddr,
	unsigned long len, int fd)
#elif defined(CONFIG_MACH_MOT)
static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info, struct file *file, unsigned long paddr,
	unsigned long len, int fd)
#else
static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info)
#endif
{
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
	struct file *file;
	unsigned long paddr;
	unsigned long kvstart;
	unsigned long len;
	int rc;
	struct msm_pmem_region *region;

	rc = get_pmem_file(info->fd, &paddr, &kvstart, &len, &file);
	if (rc < 0) {
		pr_err("%s: get_pmem_file fd %d error %d\n",
			__func__,
			info->fd, rc);
		return rc;
	}

	if (check_overlap(ptype, paddr, len) < 0)
	return -EINVAL;
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_pmem_region *region =
		kmalloc(sizeof(*region), GFP_KERNEL);
#endif
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
	CDBG("%s: type = %d, paddr = 0x%lx, vaddr = 0x%lx\n",
		__func__,
		info->type, paddr, (unsigned long)info->vaddr);
	region = kmalloc(sizeof(*region), GFP_KERNEL);

#endif
	if (!region)
		return -ENOMEM;

	INIT_HLIST_NODE(&region->list);

	region->type = info->type;
	region->vaddr = info->vaddr;
	region->paddr = paddr;
	region->len = len;
	region->file = file;
	region->y_off = info->y_off;
	region->cbcr_off = info->cbcr_off;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	region->fd = fd;
#else
	region->fd = info->fd;
#endif
	region->active = info->active;

	hlist_add_head(&(region->list), ptype);

	return 0;
}
#if defined(CONFIG_MACH_PITTSBURGH)
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	enum msm_pmem_t type, struct msm_pmem_region *reg, uint8_t maxcount)
#elif defined(CONFIG_MACH_MOT)
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	int type, struct msm_pmem_region *reg, uint8_t maxcount)
#else
/* return of 0 means failure */
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
		int pmem_type, struct msm_pmem_region *reg, uint8_t maxcount)
#endif
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct hlist_node *node;
#else
	struct hlist_node *node, *n;
#endif
	uint8_t rc = 0;

	regptr = reg;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	hlist_for_each_entry(region, node, ptype, list) {

		if ((region->type == type) &&
				(region->active)) {
			*regptr = *region;

			rc += 1;
			if (rc >= maxcount)
				break;

			regptr++;
		}
	}
#else
	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		if (region->type == pmem_type && region->active) {
			*regptr = *region;

			rc += 1;
			if (rc >= maxcount)
				break;

			regptr++;
		}
	}
#endif
	return rc;
}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static unsigned long msm_pmem_frame_ptov_lookup(unsigned long pyaddr,
	unsigned long pcbcraddr, uint32_t *yoff, uint32_t *cbcroff, int *fd,
	struct msm_device_t	*msm)
#else
static unsigned long msm_pmem_frame_ptov_lookup(struct msm_sync *sync,
	unsigned long pyaddr,
	unsigned long pcbcraddr,
	uint32_t *yoff, uint32_t *cbcroff, int *fd)
#endif
{
	struct msm_pmem_region *region;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct hlist_node *node;
	unsigned long rc = 0;
#else
	struct hlist_node *node, *n;
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	hlist_for_each_entry(region, node,
		&msm->sync.frame, list) {

		if (pyaddr == (region->paddr + region->y_off) &&
		pcbcraddr == (region->paddr + region->cbcr_off) &&
		region->active) {

			/* offset since we could pass vaddr inside
			 * a registerd pmem buffer */
			rc = (unsigned long)(region->vaddr);
			*yoff = region->y_off;
			*cbcroff = region->cbcr_off;
			*fd = region->fd;
			region->active = 0;

			return rc;
		}
	}
#else
	hlist_for_each_entry_safe(region, node, n, &sync->frame, list) {
		if (pyaddr == (region->paddr + region->y_off) &&
			pcbcraddr == (region->paddr +
			region->cbcr_off) &&
			region->active) {

			/* offset since we could pass vaddr inside
			 * a registerd pmem buffer */
			*yoff = region->y_off;
			*cbcroff = region->cbcr_off;
			*fd = region->fd;
			region->active = 0;
			return (unsigned long)(region->vaddr);
		}
	}
#endif
	return 0;
}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static unsigned long msm_pmem_stats_ptov_lookup(unsigned long addr, int *fd,
	struct msm_device_t *msm)
#else
static unsigned long msm_pmem_stats_ptov_lookup(struct msm_sync *sync,
	unsigned long addr, int *fd)
#endif

{
	struct msm_pmem_region *region;

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct hlist_node *node;
	unsigned long rc = 0;
#else
	struct hlist_node *node, *n;
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	hlist_for_each_entry(region, node,
		&msm->sync.stats, list) {

		if (addr == region->paddr &&
				region->active) {
			/* offset since we could pass vaddr inside a
			*  registered pmem buffer */
			rc = (unsigned long)(region->vaddr);
			*fd = region->fd;
			region->active = 0;
			return rc;
		}
	}
#else
	hlist_for_each_entry_safe(region, node, n, &sync->stats, list) {
		if (addr == region->paddr &&
				region->active) {
			/* offset since we could pass vaddr inside a
			*  registered pmem buffer */
			*fd = region->fd;
			region->active = 0;
			return (unsigned long)(region->vaddr);
		}
	}
#endif
	return 0;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static void msm_pmem_frame_vtop_lookup(unsigned long buffer,
	uint32_t yoff, uint32_t cbcroff, int fd, unsigned long *phyaddr,
	struct msm_device_t *msm)

#else
static unsigned long msm_pmem_frame_vtop_lookup(struct msm_sync *sync,
		unsigned long buffer,
		uint32_t yoff, uint32_t cbcroff, int fd)
#endif
{
	struct msm_pmem_region *region;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct hlist_node *node;
#else
	struct hlist_node *node, *n;
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	hlist_for_each_entry(region,
		node, &msm->sync.frame, list) {

		if (((unsigned long)(region->vaddr) == buffer) &&
				(region->y_off == yoff) &&
				(region->cbcr_off == cbcroff) &&
				(region->fd == fd) &&
				(region->active == 0)) {

			*phyaddr = region->paddr;
			region->active = 1;

			return;
		}
	}

	*phyaddr = 0;
#else
	hlist_for_each_entry_safe(region,
		node, n, &sync->frame, list) {
		if (((unsigned long)(region->vaddr) == buffer) &&
				(region->y_off == yoff) &&
				(region->cbcr_off == cbcroff) &&
				(region->fd == fd) &&
				(region->active == 0)) {

			region->active = 1;
			return region->paddr;
		}
	}
	return 0;
#endif

}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static void msm_pmem_stats_vtop_lookup(unsigned long buffer,
	int fd, unsigned long *phyaddr, struct msm_device_t *msm)
#else
static unsigned long msm_pmem_stats_vtop_lookup(
	struct msm_sync *sync,
	unsigned long buffer,
	int fd)
#endif
{
	struct msm_pmem_region *region;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct hlist_node *node;
#else
	struct hlist_node *node, *n;

#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	hlist_for_each_entry(region, node,
		&msm->sync.stats, list) {

		if (((unsigned long)(region->vaddr) == buffer) &&
				(region->fd == fd) &&
				region->active == 0) {

			*phyaddr = region->paddr;
			region->active = 1;

			return;
		}
	}
	*phyaddr = 0;
#else
	hlist_for_each_entry_safe(region, node, n, &sync->stats, list) {
		if (((unsigned long)(region->vaddr) == buffer) &&
				(region->fd == fd) && region->active == 0) {
			region->active = 1;
			return region->paddr;
		}
	}
	return 0;
#endif
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_pmem_table_del_proc(struct msm_pmem_info *pinfo,
	struct msm_device_t *msm)
#else
static int __msm_pmem_table_del(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
#endif
{
	struct msm_pmem_region *region;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct hlist_node *node;
	struct hlist_node *n;
	mutex_lock(&msm->msm_sem);
#else
	struct hlist_node *node, *n;
	int rc = 0;
#endif

	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
#if defined(CONFIG_MACH_PITTSBURGH)
	case MSM_PMEM_THUMBAIL:
#else
	case MSM_PMEM_THUMBNAIL:
#endif
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		hlist_for_each_entry_safe(region, node, n,
			&msm->sync.frame, list) {
			if (pinfo->type == region->type &&
					pinfo->vaddr == region->vaddr &&
					pinfo->fd == region->fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
#else
		hlist_for_each_entry_safe(region, node, n,
			&sync->frame, list) {
			if (pinfo->type == region->type &&
					pinfo->vaddr == region->vaddr &&
					pinfo->fd == region->fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
#endif
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		hlist_for_each_entry_safe(region, node, n,
			&msm->sync.stats, list) {

			if (pinfo->type == region->type &&
					pinfo->vaddr == region->vaddr &&
					pinfo->fd == region->fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
#else
		hlist_for_each_entry_safe(region, node, n,
			&sync->stats, list) {
			if (pinfo->type == region->type &&
					pinfo->vaddr == region->vaddr &&
					pinfo->fd == region->fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
#endif
		break;

	default:
		rc = -EINVAL;
		break;
	}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	mutex_unlock(&msm->msm_sem);
#endif
	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_pmem_table_del(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_pmem_table_del(struct msm_sync *sync, void __user *arg)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_pmem_info info;
	if (copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;

	return msm_pmem_table_del_proc(&info, msm);
#else
	struct msm_pmem_info info;
	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
	}
	return __msm_pmem_table_del(sync, &info);
#endif
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_get_frame_proc(struct msm_frame *frame,
	struct msm_device_t	*msm)
#else
static int __msm_get_frame(struct msm_sync *sync,
	struct msm_frame *frame)
#endif
{
	unsigned long flags;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct msm_queue_cmd_t *qcmd = NULL;
#else
	int rc = 0;
	struct msm_queue_cmd *qcmd = NULL;
#endif

	struct msm_vfe_phy_info *pphy;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	spin_lock_irqsave(&msm->sync.prev_frame_q_lock, flags);

	if (!list_empty(&msm->sync.prev_frame_q)) {
		qcmd = list_first_entry(&msm->sync.prev_frame_q,
			struct msm_queue_cmd_t, list);
		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock, flags);

	if (!qcmd)
		return -EAGAIN;
#else
	spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
	if (!list_empty(&sync->prev_frame_q)) {
	qcmd = list_first_entry(&sync->prev_frame_q,
		struct msm_queue_cmd, list);
		list_del_init(&qcmd->list);
	}
	spin_unlock_irqrestore(&sync->prev_frame_q_lock, flags);
	if (!qcmd) {
		pr_err("%s: no preview frame.\n", __func__);
	}

#endif
	pphy = (struct msm_vfe_phy_info *)(qcmd->command);

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	frame->buffer =
		msm_pmem_frame_ptov_lookup(pphy->y_phy,
			pphy->cbcr_phy, &(frame->y_off),
			&(frame->cbcr_off), &(frame->fd), msm);
#else
	frame->buffer =
		msm_pmem_frame_ptov_lookup(sync,
		pphy->y_phy,
		pphy->cbcr_phy, &(frame->y_off),
		&(frame->cbcr_off), &(frame->fd));
	if (!frame->buffer) {
		pr_err("%s: cannot get frame, invalid lookup address "
			"y=%x cbcr=%x offset=%d\n",
			 __func__,
			pphy->y_phy,
			pphy->cbcr_phy,
			frame->y_off);
		rc = -EINVAL;
	}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	CVBS("get_fr_proc: y= 0x%x, cbcr= 0x%x, qcmd= 0x%x, virt_addr= 0x%x\n",
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);
#else
	CDBG("__msm_get_frame: y=0x%x, cbcr=0x%x, qcmd=0x%x, virt_addr=0x%x\n",
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	kfree(qcmd->command);
#endif
	kfree(qcmd);
	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_get_frame(void __user *arg,
	struct msm_device_t	*msm)
#else
static int msm_get_frame(struct msm_sync *sync, void __user *arg)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct msm_frame frame;
#else
	int rc = 0;
	struct msm_frame frame;
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame)))
		return -EFAULT;
#else
	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	rc = msm_get_frame_proc(&frame, msm);
#else
	rc = __msm_get_frame(sync, &frame);
#endif
	if (rc < 0)
		return rc;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (msm->croplen) {
		if (frame.croplen > msm->croplen)
			return -EINVAL;
		if (copy_to_user((void *)frame.cropinfo,
				msm->cropinfo,
				msm->croplen))
		return -EFAULT;
	}
#else
	if (sync->croplen) {
		if (frame.croplen > sync->croplen) {
			pr_err("msm_get_frame: invalid frame croplen %d\n",
			frame.croplen);

			return -EINVAL;
		}
		if (copy_to_user((void *)frame.cropinfo,
			sync->cropinfo,sync->croplen)) {
				ERR_COPY_TO_USER();
				return -EFAULT;
		}
		
	}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (copy_to_user((void *)arg,
				&frame, sizeof(struct msm_frame)))
		rc = -EFAULT;
#else
	if (copy_to_user((void *)arg,
			&frame, sizeof(struct msm_frame))) {
			ERR_COPY_TO_USER();
			rc = -EFAULT;
	}
#endif

	CDBG("Got frame!!!\n");

	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_enable_vfe(void __user *arg,
	struct msm_device_t *msm)
{
	long rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
                       arg,
                       sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (msm->vfefn.vfe_enable)
		rc = msm->vfefn.vfe_enable(&cfg);

	CDBG("msm_enable_vfe:returned rc = %ld\n", rc);

	return rc;
}
#else
static int msm_enable_vfe(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
			arg,
			sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (sync->vfefn.vfe_enable)
		rc = sync->vfefn.vfe_enable(&cfg);

	CDBG("msm_enable_vfe: returned rc = %d\n", rc);
	return rc;
}

static int msm_disable_vfe(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
			arg,
			sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (sync->vfefn.vfe_disable)
		rc = sync->vfefn.vfe_disable(&cfg, NULL);

	CDBG("msm_disable_vfe: returned rc = %d\n", rc);
	return rc;
}
#endif
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
static struct msm_queue_cmd *__msm_control(struct msm_sync *sync,
		struct msm_control_device_queue *queue,
		struct msm_queue_cmd *qcmd,
		int timeout)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&sync->msg_event_q_lock, flags);
	list_add_tail(&qcmd->list, &sync->msg_event_q);
	/* wake up config thread */
	wake_up(&sync->msg_event_wait);
	spin_unlock_irqrestore(&sync->msg_event_q_lock, flags);

	if (!queue)
		return NULL;

	/* wait for config status */
	rc = wait_event_interruptible_timeout(
			queue->ctrl_status_wait,
			!list_empty_careful(&queue->ctrl_status_q),
			timeout);
	if (list_empty_careful(&queue->ctrl_status_q)) {
		if (!rc)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("msm_control: wait_event error %d\n", rc);
#if 0
			/* This is a bit scary.  If we time out too early, we
			 * will free qcmd at the end of this function, and the
			 * dsp may do the same when it does respond, so we
			 * remove the message from the source queue.
			 */
			pr_err("%s: error waiting for ctrl_status_q: %d\n",
				__func__, rc);
			spin_lock_irqsave(&sync->msg_event_q_lock, flags);
			list_del_init(&qcmd->list);
			spin_unlock_irqrestore(&sync->msg_event_q_lock, flags);
#endif
			return ERR_PTR(rc);
		}
	}

	/* control command status is ready */
	spin_lock_irqsave(&queue->ctrl_status_q_lock, flags);
	BUG_ON(list_empty(&queue->ctrl_status_q));
	qcmd = list_first_entry(&queue->ctrl_status_q,
			struct msm_queue_cmd, list);
	list_del_init(&qcmd->list);
	spin_unlock_irqrestore(&queue->ctrl_status_q_lock, flags);

	return qcmd;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int msm_ctrl_stats_pending(struct msm_device_t *msm)
{
	unsigned long flags;
	int yes = 0;

	spin_lock_irqsave(&msm->sync.ctrl_status_lock,
		flags);

	yes = !list_empty(&msm->sync.ctrl_status_queue);

	spin_unlock_irqrestore(&msm->sync.ctrl_status_lock,
		flags);

	CVBS("msm_ctrl_stats_pending, yes = %d\n", yes);
	return yes;
}
static long msm_control(void __user *arg,
	struct msm_device_t *msm)
{
	unsigned long flags;
	int timeout;
	long rc = 0;

	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_queue_cmd_t *qcmd = NULL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(ctrlcmd_t))) {
		rc = -EFAULT;
		goto end;
	}

	ctrlcmd = kmalloc(sizeof(ctrlcmd_t), GFP_ATOMIC);
	if (!ctrlcmd) {
		CDBG("msm_control: cannot allocate buffer ctrlcmd\n");
		rc = -ENOMEM;
		goto end;
	}

	ctrlcmd->value = kmalloc(ctrlcmd_t.length, GFP_ATOMIC);
	if (!ctrlcmd->value) {
		CDBG("msm_control: cannot allocate buffer ctrlcmd->value\n");
		rc = -ENOMEM;
		goto no_mem;
	}

	if (copy_from_user(ctrlcmd->value,
				ctrlcmd_t.value,
				ctrlcmd_t.length)) {
		rc = -EFAULT;
		goto fail;
	}

	ctrlcmd->type = ctrlcmd_t.type;
	ctrlcmd->length = ctrlcmd_t.length;

	qcmd = kmalloc(sizeof(struct msm_queue_cmd_t), GFP_ATOMIC);
	if (!qcmd) {
		rc = -ENOMEM;
		goto fail;
	}

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock, flags);
	qcmd->type = MSM_CAM_Q_CTRL;
	qcmd->command = ctrlcmd;
	list_add_tail(&qcmd->list, &msm->sync.msg_event_queue);
	/* wake up config thread */
	wake_up(&msm->sync.msg_event_wait);
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock, flags);

    CDBG("%s: type=%d\n", __func__, ctrlcmd_t.type);

	/* wait for config status */
	timeout = (int)ctrlcmd_t.timeout_ms;
	CDBG("%s: timeout=%d\n", __func__, timeout);
	if (timeout > 0) {
		rc = wait_event_timeout(msm->sync.ctrl_status_wait,
					msm_ctrl_stats_pending(msm),
					msecs_to_jiffies(timeout));

		if (rc == 0) {
			CDBG("%s: command type %d TIMED OUT\n", __func__, ctrlcmd_t.type);
			rc = -ETIMEDOUT;
			goto timeout;
		} else {
    		CDBG("%s: command type %d completed (%ld remaining)\n", __func__, ctrlcmd_t.type, rc);
        }
	} else
		rc = wait_event_interruptible(msm->sync.ctrl_status_wait,
			msm_ctrl_stats_pending(msm));

	if (rc < 0) {
		rc = -EAGAIN;
		goto timeout;
	}

	/* control command status is ready */
	spin_lock_irqsave(&msm->sync.ctrl_status_lock, flags);
	if (!list_empty(&msm->sync.ctrl_status_queue)) {
		qcmd = list_first_entry(&msm->sync.ctrl_status_queue,
			struct msm_queue_cmd_t, list);

		if (!qcmd) {
			spin_unlock_irqrestore(&msm->sync.ctrl_status_lock,
				flags);
			rc = -EAGAIN;
			goto fail;
		}

		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.ctrl_status_lock, flags);

	if (!qcmd->command) {
		ctrlcmd_t.type = 0xFFFF;
		ctrlcmd_t.length = 0xFFFF;
		ctrlcmd_t.status = 0xFFFF;
	} else {

		CDBG("%s: length = %d\n", __func__,
			((struct msm_ctrl_cmd *)(qcmd->command))->length);
		ctrlcmd_t.type =
			((struct msm_ctrl_cmd *)(qcmd->command))->type;

		ctrlcmd_t.length =
			((struct msm_ctrl_cmd *)(qcmd->command))->length;

		ctrlcmd_t.status =
			((struct msm_ctrl_cmd *)(qcmd->command))->status;

		if (ctrlcmd_t.length > 0) {
			if (copy_to_user(ctrlcmd_t.value,
			((struct msm_ctrl_cmd *)(qcmd->command))->value,
			((struct msm_ctrl_cmd *)(qcmd->command))->length)) {

				CDBG("copy_to_user value failed!\n");
				rc = -EFAULT;
				goto end;
			}

			kfree(((struct msm_ctrl_cmd *)
				(qcmd->command))->value);
		}

		if (copy_to_user((void *)arg,
				&ctrlcmd_t,
				sizeof(struct msm_ctrl_cmd))) {
			CDBG("copy_to_user ctrlcmd failed!\n");
			rc = -EFAULT;
			goto end;
		}
	}

	goto end;

fail:
	if (ctrlcmd)
		kfree(ctrlcmd->value);

no_mem:
	kfree(ctrlcmd);

end:
	if (qcmd) {
		kfree(qcmd->command);
		kfree(qcmd);
	}

timeout:
	CDBG("msm_control: end rc = %ld\n", rc);
	return rc;
}
#else
static int msm_control(struct msm_control_device *ctrl_pmsm,
			int block,
			void __user *arg)
{
	int rc = 0;

	struct msm_sync *sync = ctrl_pmsm->pmsm->sync;
	struct msm_ctrl_cmd udata, *ctrlcmd;
	struct msm_queue_cmd *qcmd = NULL, *qcmd_temp;

	if (copy_from_user(&udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto end;
	}

	qcmd = kmalloc(sizeof(struct msm_queue_cmd) +
				sizeof(struct msm_ctrl_cmd) + udata.length,
				GFP_KERNEL);
	if (!qcmd) {
		pr_err("msm_control: cannot allocate buffer\n");
		rc = -ENOMEM;
		goto end;
	}

	qcmd->type = MSM_CAM_Q_CTRL;
	qcmd->command = ctrlcmd = (struct msm_ctrl_cmd *)(qcmd + 1);
	*ctrlcmd = udata;
	ctrlcmd->value = ctrlcmd + 1;

	if (udata.length) {
		if (copy_from_user(ctrlcmd->value,
				udata.value, udata.length)) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
			goto end;
		}
	}

	if (!block) {
		/* qcmd will be set to NULL */
		qcmd = __msm_control(sync, NULL, qcmd, 0);
		goto end;
	}

	qcmd_temp = __msm_control(sync,
				  &ctrl_pmsm->ctrl_q,
				  qcmd, MAX_SCHEDULE_TIMEOUT);

	if (IS_ERR(qcmd_temp)) {
		rc = PTR_ERR(qcmd_temp);
		goto end;
	}
	qcmd = qcmd_temp;

	if (qcmd->command) {
		void __user *to = udata.value;
		udata = *(struct msm_ctrl_cmd *)qcmd->command;
		if (udata.length > 0) {
			if (copy_to_user(to,
					 udata.value,
					 udata.length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto end;
			}
		}
		udata.value = to;

		if (copy_to_user((void *)arg, &udata,
				sizeof(struct msm_ctrl_cmd))) {
			ERR_COPY_TO_USER();
			rc = -EFAULT;
			goto end;
		}
	}

end:
	/* Note: if we get here as a result of an error, we will free the
	 * qcmd that we kmalloc() in this function.  When we come here as
	 * a result of a successful completion, we are freeing the qcmd that
	 * we dequeued from queue->ctrl_status_q.
	 */
	if (qcmd)
	kfree(qcmd);

	CDBG("msm_control: end rc = %d\n", rc);
	return rc;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int msm_stats_pending(struct msm_device_t *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd_t *qcmd = NULL;

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock, flags);

	if (!list_empty(&msm->sync.msg_event_queue)) {

		qcmd = list_first_entry(&msm->sync.msg_event_queue,
			struct msm_queue_cmd_t, list);

		if (qcmd) {
			if ((qcmd->type  == MSM_CAM_Q_CTRL)    ||
				(qcmd->type  == MSM_CAM_Q_VFE_EVT) ||
				(qcmd->type  == MSM_CAM_Q_VFE_MSG) ||
				(qcmd->type  == MSM_CAM_Q_V4L2_REQ)) {
				yes = 1;
			}
		}
	}
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock, flags);

	CVBS("msm_stats_pending, yes = %d\n", yes);
	return yes;
}

static long msm_get_stats(void __user *arg,
	struct msm_device_t *msm)
{
	unsigned long flags;
	int           timeout;
	long          rc = 0;

	struct msm_stats_event_ctrl se;

	struct msm_queue_cmd_t *qcmd = NULL;
	struct msm_ctrl_cmd  *ctrl = NULL;
	struct msm_vfe_resp_t  *data = NULL;
	struct msm_stats_buf stats;

	if (copy_from_user(&se, arg,
				sizeof(struct msm_stats_event_ctrl)))
		return -EFAULT;

	timeout = (int)se.timeout_ms;

	if (timeout > 0) {
		rc =
			wait_event_timeout(
				msm->sync.msg_event_wait,
				msm_stats_pending(msm),
				msecs_to_jiffies(timeout));

		if (rc == 0) {
			CDBG("%s: timed out (%d ms)\n", __func__, timeout);
			return -ETIMEDOUT;
		}
	} else {
		rc = wait_event_interruptible(msm->sync.msg_event_wait,
			msm_stats_pending(msm));
	}

	if (rc < 0) {
		CDBG("%s: failed rc = %ld\n", __func__, rc);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock, flags);
	if (!list_empty(&msm->sync.msg_event_queue)) {
		qcmd = list_first_entry(&msm->sync.msg_event_queue,
				struct msm_queue_cmd_t, list);

		if (!qcmd) {
			spin_unlock_irqrestore(
				&msm->sync.msg_event_queue_lock, flags);
			return -EAGAIN;
		}

		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock, flags);

	CVBS("%s: received from DSP: %d\n", __func__, qcmd->type);

	switch (qcmd->type) {
	case MSM_CAM_Q_VFE_EVT:
		CVBS("%s: MSM_CAM_Q_VFE_EVT\n", __func__);
	case MSM_CAM_Q_VFE_MSG:
		CVBS("%s: MSM_CAM_Q_VFE_MSG\n", __func__);

		data = (struct msm_vfe_resp_t *)(qcmd->command);

		/* adsp event and message */
		se.resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		se.stats_event.type   = data->evt_msg.type;
		se.stats_event.msg_id = data->evt_msg.msg_id;
		se.stats_event.len    = data->evt_msg.len;

		CVBS("%s: stats_event.len=%d, .msg_id=%d\n",__func__, se.stats_event.len, se.stats_event.msg_id);

		if ((data->type == VFE_MSG_STATS_AF) ||
				(data->type == VFE_MSG_STATS_WE)) {

			stats.buffer =
			msm_pmem_stats_ptov_lookup(data->phy.sbuf_phy,
				&(stats.fd), msm);

			if (copy_to_user((void *)(se.stats_event.data),
				&stats, sizeof(struct msm_stats_buf))) {

				rc = -EFAULT;
				goto failure;
			}
		} else if ((data->evt_msg.len > 0) &&
				(data->type == VFE_MSG_GENERAL)) {

			if (copy_to_user((void *)(se.stats_event.data),
					data->evt_msg.data,
					data->evt_msg.len))
				rc = -EFAULT;

		} else if (data->type == VFE_MSG_OUTPUT1 ||
			data->type == VFE_MSG_OUTPUT2) {

			uint32_t pp_en;
			struct msm_postproc buf;
			mutex_lock(&msm->pict_pp_lock);
			pp_en = msm->pict_pp;
			mutex_unlock(&msm->pict_pp_lock);
			if (pp_en & PP_PREV) {
				CDBG("Started Preview post processing. pp_en = %d \n", pp_en);
				buf.fmain.buffer =
					msm_pmem_frame_ptov_lookup(data->phy.y_phy,
					data->phy.cbcr_phy, &buf.fmain.y_off,
					&buf.fmain.cbcr_off, &buf.fmain.fd, msm);
				if (buf.fmain.buffer) {
          CDBG("%s: Copy_to_user: buf=0x%lx fd=%d y_o=%d c_o=%d\n", __func__,
            buf.fmain.buffer, buf.fmain.fd, buf.fmain.y_off, buf.fmain.cbcr_off);
			if (copy_to_user((void *)(se.stats_event.data),
						&(buf.fmain),
						sizeof(struct msm_frame)))
							rc = -EFAULT;
					} else
							rc = -EFAULT;
			} else {
				if (copy_to_user((void *)(se.stats_event.data),
					data->extdata,
					data->extlen))
				rc = -EFAULT;
			}
		} else if (data->type == VFE_MSG_SNAPSHOT) {

			uint32_t pp_en;
			struct msm_postproc buf;
			struct msm_pmem_region region;
            memset(&region, 0, sizeof(region));

			mutex_lock(&msm->pict_pp_lock);
			pp_en = msm->pict_pp;
			mutex_unlock(&msm->pict_pp_lock);

			if (pp_en & PP_SNAP) {
				buf.fmnum =
					msm_pmem_region_lookup(
						&msm->sync.frame,
						MSM_PMEM_MAINIMG,
						&region, 1);

				if (buf.fmnum == 1) {
					buf.fmain.buffer =
					(unsigned long)region.vaddr;

					buf.fmain.y_off  = region.y_off;
					buf.fmain.cbcr_off = region.cbcr_off;
					buf.fmain.fd = region.fd;
				} else {
					buf.fmnum =
					msm_pmem_region_lookup(
						&msm->sync.frame,
						MSM_PMEM_RAW_MAINIMG,
						&region, 1);

					if (buf.fmnum == 1) {
						buf.fmain.path =
							MSM_FRAME_PREV_2;
						buf.fmain.buffer =
						(unsigned long)region.vaddr;

						buf.fmain.fd = region.fd;
					}
				}

				if (copy_to_user((void *)(se.stats_event.data),
					&buf, sizeof(struct msm_postproc))) {

					rc = -EFAULT;
					goto failure;
				}
			}

			CDBG("SNAPSHOT copy_to_user!\n");
		}
		break;

	case MSM_CAM_Q_CTRL:{
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CVBS("%s: MSM_CAM_Q_CTRL: length=%d\n", __func__, ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
						ctrl->value,
						ctrl->length)) {

				rc = -EFAULT;
				goto failure;
			}
		}

		se.resptype = MSM_CAM_RESP_CTRL;

		/* what to control */
		se.ctrl_cmd.type = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
	} /* MSM_CAM_Q_CTRL */
		break;

	case MSM_CAM_Q_V4L2_REQ: {
		/* control command from v4l2 client */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CVBS("%s: MSM_CAM_Q_V4L2_REQ length=%d\n", __func__, ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
					ctrl->value, ctrl->length)) {

				rc = -EFAULT;
				goto failure;
			}
		}

		/* 2 tells config thread this is v4l2 request */
		se.resptype = MSM_CAM_RESP_V4L2;

		/* what to control */
		se.ctrl_cmd.type   = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
	} /* MSM_CAM_Q_V4L2_REQ */
		break;

	default:
		rc = -EFAULT;
		goto failure;
	} /* switch qcmd->type */

	if (copy_to_user((void *)arg, &se, sizeof(se)))
		rc = -EFAULT;

failure:
	if (qcmd) {

		if (qcmd->type == MSM_CAM_Q_VFE_MSG)
			kfree(((struct msm_vfe_resp_t *)
				(qcmd->command))->evt_msg.data);

		kfree(qcmd->command);
		kfree(qcmd);
	}

	if (rc < 0) {
        CDBG("%s: failed; rc = %ld\n", __func__, rc);
    } else {
    	CVBS("%s: success; rc = %ld\n", __func__, rc);
    }
	return rc;
}
#else

static int msm_get_stats(struct msm_sync *sync, void __user *arg)
{
	unsigned long flags;
	int timeout;
	int rc = 0;

	struct msm_stats_event_ctrl se;
	struct msm_queue_cmd *qcmd_frame = NULL;
	struct msm_vfe_phy_info *fphy;

	struct msm_queue_cmd *qcmd = NULL;
	struct msm_ctrl_cmd  *ctrl = NULL;
	struct msm_vfe_resp  *data = NULL;
	struct msm_stats_buf stats;

	if (copy_from_user(&se, arg,
			sizeof(struct msm_stats_event_ctrl))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	timeout = (int)se.timeout_ms;

	CDBG("msm_get_stats timeout %d\n", timeout);
	rc = wait_event_interruptible_timeout(
			sync->msg_event_wait,
			!list_empty_careful(&sync->msg_event_q),
			msecs_to_jiffies(timeout));
	if (list_empty_careful(&sync->msg_event_q)) {
		if (rc == 0)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("msm_get_stats error %d\n", rc);
			return rc;
		}
	}
	CDBG("msm_get_stats returned from wait: %d\n", rc);

	spin_lock_irqsave(&sync->msg_event_q_lock, flags);
	BUG_ON(list_empty(&sync->msg_event_q));
	qcmd = list_first_entry(&sync->msg_event_q,
			struct msm_queue_cmd, list);
	list_del_init(&qcmd->list);
	spin_unlock_irqrestore(&sync->msg_event_q_lock, flags);

	CDBG("=== received from DSP === %d\n", qcmd->type);

	switch (qcmd->type) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		data = (struct msm_vfe_resp *)(qcmd->command);

		/* adsp event and message */
		se.resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		se.stats_event.type   = data->evt_msg.type;
		se.stats_event.msg_id = data->evt_msg.msg_id;
		se.stats_event.len    = data->evt_msg.len;

		CDBG("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CDBG("length = %d\n", se.stats_event.len);
		CDBG("msg_id = %d\n", se.stats_event.msg_id);

		if ((data->type == VFE_MSG_STATS_AF) ||
				(data->type == VFE_MSG_STATS_WE)) {

			stats.buffer =
			msm_pmem_stats_ptov_lookup(sync,
					data->phy.sbuf_phy,
					&(stats.fd));
			if (!stats.buffer) {
				pr_err("%s: msm_pmem_stats_ptov_lookup error\n",
					__FUNCTION__);
				rc = -EINVAL;
				goto failure;
			}

			if (copy_to_user((void *)(se.stats_event.data),
					&stats,
					sizeof(struct msm_stats_buf))) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		} else if ((data->evt_msg.len > 0) &&
				(data->type == VFE_MSG_GENERAL)) {
			if (copy_to_user((void *)(se.stats_event.data),
					data->evt_msg.data,
					data->evt_msg.len)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
			}
		} else if (data->type == VFE_MSG_OUTPUT1 ||
			data->type == VFE_MSG_OUTPUT2) {
			if (sync->pict_pp & PP_PREV) {
				struct msm_postproc buf;

				sync->pp_sync_flag = 0;
				CDBG("%s: Preview PP. sync->pict_pp = %d \n",
					__func__, sync->pict_pp);

				buf.fmain.buffer =
					msm_pmem_frame_ptov_lookup(sync,
					data->phy.y_phy,
					data->phy.cbcr_phy,
					&(buf.fmain.y_off),
					&(buf.fmain.cbcr_off),
					&(buf.fmain.fd));
				if (buf.fmain.buffer) {
					CDBG("%s: buf=%ld fd=%d\n",
						__func__, buf.fmain.buffer,
						buf.fmain.fd);
					if (copy_to_user((void *)
						(se.stats_event.data),
						&(buf.fmain),
						sizeof(struct msm_frame))) {
						CDBG("%s: failed at line=%d\n",
							__func__, __LINE__);
						rc = -EFAULT;
					}
				} else {
					CDBG("%s: failed at line=%d\n",
						__func__, __LINE__);
					rc = -EFAULT;
				}

			} else {
				CDBG("%s: msm->pp_sync_flag = %d \n",
					 __func__, sync->pp_sync_flag);

				if (sync->pp_sync_flag == 1) {
					fphy = kzalloc(
					sizeof(struct msm_vfe_phy_info),
						GFP_ATOMIC);
					if (!fphy) {
						rc = -EFAULT;
						goto failure;
					}

					qcmd_frame =
					kmalloc(sizeof(struct msm_queue_cmd),
						GFP_ATOMIC);
					if (!qcmd_frame) {
						rc = -EFAULT;
						goto failure;
					}

					*fphy = data->phy;
					qcmd_frame->type = MSM_CAM_Q_VFE_MSG;
					qcmd_frame->command = fphy;

					CDBG("%s: phy_y=0x%x phy_cbcr=0x%x\n",
						__func__, fphy->y_phy,
						fphy->cbcr_phy);

					spin_lock_irqsave(
						&sync->prev_frame_q_lock,
						flags);

					list_add_tail(&qcmd_frame->list,
						&sync->prev_frame_q);

					spin_unlock_irqrestore(
						&sync->prev_frame_q_lock,
						flags);

					wake_up(&sync->prev_frame_wait);
					sync->pp_sync_flag = 0;
					CDBG("%s: waked up frame thread\n",
						__func__);
				} else {
					CDBG("%s:For config thread.\n",
						__func__);
					if (copy_to_user(
						(void *)(se.stats_event.data),
							data->extdata,
							data->extlen)) {
						ERR_COPY_TO_USER();
						rc = -EFAULT;
					}
				}

			}
		} else if (data->type == VFE_MSG_SNAPSHOT) {
			if (sync->pict_pp & PP_SNAP) {
				struct msm_postproc buf;
				struct msm_pmem_region region;
				buf.fmnum = msm_pmem_region_lookup(&sync->frame,
						MSM_PMEM_MAINIMG,
						&region, 1);
				if (buf.fmnum == 1) {
					buf.fmain.buffer =
						(unsigned long)region.vaddr;
					buf.fmain.y_off  = region.y_off;
					buf.fmain.cbcr_off = region.cbcr_off;
					buf.fmain.fd = region.fd;
				} else {
					buf.fmnum = msm_pmem_region_lookup(
							&sync->frame,
							MSM_PMEM_RAW_MAINIMG,
							&region, 1);
					if (buf.fmnum == 1) {
						buf.fmain.path =
							MSM_FRAME_PREV_2;
						buf.fmain.buffer =
						(unsigned long)region.vaddr;

						buf.fmain.fd = region.fd;
					} else {
						pr_err("%s: pmem lookup fail\n",
							__func__);
						rc = -EINVAL;
					}
				}

				if (copy_to_user(
					(void *)(se.stats_event.data), &buf,
						sizeof(buf))) {
					ERR_COPY_TO_USER();
					rc = -EFAULT;
					goto failure;
				}
			}
			CDBG("snapshot copy_to_user!\n");
		}
		break;

	case MSM_CAM_Q_CTRL:
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CDBG("length = %d\n", ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
						ctrl->value,
						ctrl->length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		}

		se.resptype = MSM_CAM_RESP_CTRL;

		/* what to control */
		se.ctrl_cmd.type = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
		se.ctrl_cmd.resp_fd = ctrl->resp_fd;
		break;

	case MSM_CAM_Q_V4L2_REQ:
		/* control command from v4l2 client */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CDBG("length = %d\n", ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
					ctrl->value, ctrl->length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		}

		/* 2 tells config thread this is v4l2 request */
		se.resptype = MSM_CAM_RESP_V4L2;

		/* what to control */
		se.ctrl_cmd.type   = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
		break;

	default:
		rc = -EFAULT;
		goto failure;
	} /* switch qcmd->type */

	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

failure:
	if (qcmd)
	kfree(qcmd);

	CDBG("msm_get_stats: %d\n", rc);
	return rc;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_ctrl_cmd_done(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_ctrl_cmd_done(struct msm_control_device *ctrl_pmsm,
			void __user *arg)
#endif
{
	unsigned long flags;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_queue_cmd_t *qcmd = NULL;
#else
	int rc = 0;
	struct msm_ctrl_cmd udata, *ctrlcmd;
	struct msm_queue_cmd *qcmd = NULL;

#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd)))
		return -EFAULT;
#else
	if (copy_from_user(&udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto end;
	}

#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!ctrlcmd) {
		rc = -ENOMEM;
		goto end;
	}
#else
	qcmd = kmalloc(sizeof(struct msm_queue_cmd) +
		sizeof(struct msm_ctrl_cmd) + udata.length,
		GFP_KERNEL);
	if (!qcmd) {
		rc = -ENOMEM;
		goto end;
	}
	qcmd->command = ctrlcmd = (struct msm_ctrl_cmd *)(qcmd + 1);
	*ctrlcmd = udata;
	if (udata.length > 0) {
		ctrlcmd->value = ctrlcmd + 1;
#endif


#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (ctrlcmd_t.length > 0) {
		ctrlcmd->value = kmalloc(ctrlcmd_t.length, GFP_ATOMIC);
		if (!ctrlcmd->value) {
			rc = -ENOMEM;
			goto no_mem;
		}

		if (copy_from_user(ctrlcmd->value,
					(void *)ctrlcmd_t.value,
					ctrlcmd_t.length)) {

			rc = -EFAULT;
			goto fail;
		}
#else
		if (copy_from_user(ctrlcmd->value,
				(void *)udata.value,
				udata.length)) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
			kfree(qcmd);
			goto end;
		}

#endif
	} else
		ctrlcmd->value = NULL;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	ctrlcmd->type = ctrlcmd_t.type;
	ctrlcmd->length = ctrlcmd_t.length;
	ctrlcmd->status = ctrlcmd_t.status;

	qcmd = kmalloc(sizeof(*qcmd), GFP_ATOMIC);
	if (!qcmd) {
		rc = -ENOMEM;
		goto fail;
	}

	qcmd->command = (void *)ctrlcmd;

	goto end;

fail:
	kfree(ctrlcmd->value);
no_mem:
	kfree(ctrlcmd);
#endif
end:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	CDBG("msm_ctrl_cmd_done: end rc = %ld\n", rc);
#else
	CDBG("msm_ctrl_cmd_done: end rc = %d\n", rc);
#endif
	if (rc == 0) {
		/* wake up control thread */
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		spin_lock_irqsave(&msm->sync.ctrl_status_lock, flags);
		list_add_tail(&qcmd->list, &msm->sync.ctrl_status_queue);
		spin_unlock_irqrestore(&msm->sync.ctrl_status_lock, flags);
		wake_up(&msm->sync.ctrl_status_wait);
#else
		spin_lock_irqsave(&ctrl_pmsm->ctrl_q.ctrl_status_q_lock, flags);
		list_add_tail(&qcmd->list, &ctrl_pmsm->ctrl_q.ctrl_status_q);
		wake_up(&ctrl_pmsm->ctrl_q.ctrl_status_wait);
		spin_unlock_irqrestore(
		&ctrl_pmsm->ctrl_q.ctrl_status_q_lock, flags);
#endif
	}

	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_config_vfe(void __user *arg,
	struct msm_device_t	*msm)
{
	struct msm_vfe_cfg_cmd cfgcmd_t;
	struct msm_pmem_region region[8];
	struct axidata axi_data;
	long rc = 0;

	memset(&axi_data, 0, sizeof(axi_data));

	if (copy_from_user(&cfgcmd_t, arg, sizeof(cfgcmd_t)))
		return -EFAULT;

	if (cfgcmd_t.cmd_type == CMD_STATS_ENABLE) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.stats,
		MSM_PMEM_AEC_AWB, &region[0],
		NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		axi_data.region = &region[0];

	} else if (cfgcmd_t.cmd_type == CMD_STATS_AF_ENABLE) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.stats,
			MSM_PMEM_AF, &region[0],
			NUM_AF_STAT_OUTPUT_BUFFERS);
		axi_data.region = &region[0];
	}

	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(&cfgcmd_t, &(axi_data));

	return rc;
}

#else

static int msm_config_vfe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;
	struct msm_pmem_region region[8];
	struct axidata axi_data;
	void *data = NULL;
	int rc = -EIO;

	memset(&axi_data, 0, sizeof(axi_data));

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	switch (cfgcmd.cmd_type) {
	case CMD_STATS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats,
					MSM_PMEM_AEC_AWB, &region[0],
					NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->stats,
					MSM_PMEM_AF, &region[axi_data.bufnum1],
					NUM_AF_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1 || !axi_data.bufnum2) {
			pr_err("%s: pmem region lookup error\n", __func__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		data = &axi_data;
		break;
	case CMD_STATS_AF_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats,
					MSM_PMEM_AF, &region[0],
					NUM_AF_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s: pmem region lookup error\n", __func__);
			pr_err("%s %d: pmem region lookup error\n",
				__FUNCTION__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		data = &axi_data;
		break;
	case CMD_STATS_AEC_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats,
			MSM_PMEM_AEC_AWB, &region[0],
			NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		axi_data.region = &region[0];
    data = &axi_data;
    break;
	case CMD_GENERAL:
	case CMD_STATS_DISABLE:
		break;
	default:
		pr_err("%s: unknown command type %d\n",
			__FUNCTION__, cfgcmd.cmd_type);
		return -EINVAL;
	}


	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(&cfgcmd, data);

	return rc;
}
#endif
#if defined(CONFIG_MACH_PITTSBURGH)
static long msm_frame_axi_cfg(struct msm_vfe_cfg_cmd *cfgcmd_t,
	struct msm_device_t *msm)
#elif defined(CONFIG_MACH_MOT)
static long msm_frame_axi_cfg(struct msm_vfe_cfg_cmd *cfgcmd_t,
	struct msm_device_t *msm)
#else
static int msm_frame_axi_cfg(struct msm_sync *sync,
	struct msm_vfe_cfg_cmd *cfgcmd)
#endif
{
#if defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct axidata_t axi_data;
	enum msm_pmem_t mtype;
#elif defined(CONFIG_MACH_MOT)
	long rc = 0;
	struct axidata axi_data;
	int mtype;
#else
	int rc = -EIO;
	int pmem_type;
	struct axidata axi_data;
	void *data = &axi_data;
#endif
	struct msm_pmem_region region[8];

	memset(&axi_data, 0, sizeof(axi_data));

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	switch (cfgcmd_t->cmd_type)
#else
	switch (cfgcmd->cmd_type)
#endif
	{
	case CMD_AXI_CFG_OUT1:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		mtype = MSM_PMEM_OUTPUT1;
#else
		pmem_type = MSM_PMEM_OUTPUT1;
#endif
		axi_data.bufnum1 =
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
#else
			 msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
#endif

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
			 __FUNCTION__, __LINE__);
		return -EINVAL;
		}
#endif
		break;

	case CMD_AXI_CFG_OUT2:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		mtype = MSM_PMEM_OUTPUT2;
#else
		pmem_type = MSM_PMEM_OUTPUT2;
#endif
		axi_data.bufnum2 =
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
#else
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
#endif
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error (empty %d)\n",
				__FUNCTION__, __LINE__,
				hlist_empty(&sync->frame));
				return -EINVAL;
		}
#endif

		break;

	case CMD_AXI_CFG_SNAP_O1_AND_O2:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		mtype = MSM_PMEM_THUMBNAIL;
#else
		pmem_type = MSM_PMEM_THUMBNAIL;
#endif
		axi_data.bufnum1 =
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
#else
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
#endif
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__FUNCTION__, __LINE__);
				return -EINVAL;
		}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		mtype = MSM_PMEM_MAINIMG;
#else
		pmem_type = MSM_PMEM_MAINIMG;
#endif
		axi_data.bufnum2 =
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[axi_data.bufnum1], 8);
#else
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[axi_data.bufnum1], 8);
#endif
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__FUNCTION__, __LINE__);
				return -EINVAL;
		}
#endif
		break;

	case CMD_RAW_PICT_AXI_CFG:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		mtype = MSM_PMEM_RAW_MAINIMG;
#else
		pmem_type = MSM_PMEM_RAW_MAINIMG;
#endif
		axi_data.bufnum2 =
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
#else
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
#endif
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
			__FUNCTION__, __LINE__);
			return -EINVAL;
		}
		break;
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
	case CMD_GENERAL:
		data = NULL;
		break;
#endif
	default:
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
		pr_err("%s: unknown command type %d\n",
			__FUNCTION__, cfgcmd->cmd_type);
			return -EINVAL;
#endif
		break;
	}

	axi_data.region = &region[0];

	/* send the AXI configuration command to driver */
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(cfgcmd_t, &axi_data);
#else
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, data);
#endif

	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_get_sensor_info(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_get_sensor_info(struct msm_sync *sync, void __user *arg)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_device_platform_data *pdata;
#else

	int rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_sensor_info *sdata;
#endif

	if (copy_from_user(&info,
				arg,
				sizeof(struct msm_camsensor_info))) {
				ERR_COPY_FROM_USER();
				return -EFAULT;
		}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	pdata = msm->pdev->dev.platform_data;
	CDBG("%s: sensor_name '%s'\n", __func__, pdata->sinfo[msm->sidx].sensor_name);
#else
	sdata = sync->pdev->dev.platform_data;
	CDBG("%s: sensor_name '%s'\n", __func__, sdata->sensor_name);
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	memcpy(&info.name[0],
		pdata->sinfo[msm->sidx].sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled =
		(pdata->sinfo[msm->sidx].flash_type != MSM_CAMERA_FLASH_NONE);
#else
	memcpy(&info.name[0],
		sdata->sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled = sdata->flash_type != MSM_CAMERA_FLASH_NONE;
#endif

	if (copy_to_user((void *)arg,
			&info,
			sizeof(struct msm_camsensor_info))) {
			ERR_COPY_TO_USER();

		rc = -EFAULT;
	}

	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_put_frame_buf_proc(struct msm_frame *pb,
	struct msm_device_t *msm)
#else
static int __msm_put_frame_buf(struct msm_sync *sync,
		struct msm_frame *pb)
#endif
{
	unsigned long pphy;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_vfe_cfg_cmd cfgcmd_t;

	long rc = 0;
#else
	struct msm_vfe_cfg_cmd cfgcmd;
	int rc = -EIO;
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	msm_pmem_frame_vtop_lookup(pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd, &pphy, msm);

	CVBS("rel: vaddr = 0x%lx, paddr = 0x%lx\n",
		pb->buffer, pphy);
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (pphy != 0) {

		cfgcmd_t.cmd_type = CMD_FRAME_BUF_RELEASE;
		cfgcmd_t.value    = (void *)pb;

		if (msm->vfefn.vfe_config)
			rc =
				msm->vfefn.vfe_config(&cfgcmd_t, &pphy);
	} else
		rc = -EFAULT;
#else
	pphy = msm_pmem_frame_vtop_lookup(sync,
		pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd);

	if (pphy != 0) {
		CDBG("rel: vaddr = 0x%lx, paddr = 0x%lx\n",
			pb->buffer, pphy);
			cfgcmd.cmd_type = CMD_FRAME_BUF_RELEASE;
			cfgcmd.value    = (void *)pb;
		if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
	} else {
		pr_err("i%s: msm_pmem_frame_vtop_lookup failed\n",
			__FUNCTION__);
			rc = -EINVAL;
	}
#endif

	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_put_frame_buffer(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_put_frame_buffer(struct msm_sync *sync, void __user *arg)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_frame buf_t;
#else
	struct msm_frame buf_t;
#endif


#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame)))
		return -EFAULT;

	return msm_put_frame_buf_proc(&buf_t, msm);
#else
	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}
	return __msm_put_frame_buf(sync, &buf_t);
#endif
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_register_pmem_proc(struct msm_pmem_info *pinfo,
	struct msm_device_t *msm)
#else
static int __msm_register_pmem(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	unsigned long paddr, len, rc = 0;
	struct file   *file;
	unsigned long vstart;
	mutex_lock(&msm->msm_sem);

	CVBS("%s: type = %d, paddr = 0x%lx, vaddr = 0x%lx\n", __func__,
		pinfo->type, paddr, (unsigned long)pinfo->vaddr);

	get_pmem_file(pinfo->fd, &paddr, &vstart, &len, &file);
#else
	int rc = 0;
#endif

	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
#if defined(CONFIG_MACH_PITTSBURGH)
	case MSM_PMEM_THUMBAIL:
#else
	case MSM_PMEM_THUMBNAIL:
#endif
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		rc = msm_pmem_table_add(&msm->sync.frame,
			pinfo, file, paddr, len, pinfo->fd);
#else
		rc = msm_pmem_table_add(&sync->frame, pinfo);
#endif
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		rc = msm_pmem_table_add(&msm->sync.stats,
			pinfo, file, paddr, len, pinfo->fd);
#else
		rc = msm_pmem_table_add(&sync->stats, pinfo);
#endif
		break;

	default:
		rc = -EINVAL;
		break;
	}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	mutex_unlock(&msm->msm_sem);
#endif
	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_register_pmem(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_register_pmem(struct msm_sync *sync, void __user *arg)
#endif

{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_pmem_info info;
#else
	struct msm_pmem_info info;
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;
#else
	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	return msm_register_pmem_proc(&info, msm);
#else
	return __msm_register_pmem(sync, &info);
#endif
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_stats_axi_cfg(struct msm_vfe_cfg_cmd *cfgcmd_t,
	struct msm_device_t *msm)
#else
static int msm_stats_axi_cfg(struct msm_sync *sync,
	struct msm_vfe_cfg_cmd *cfgcmd)
#endif
{
#if defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct axidata_t axi_data;
	enum msm_pmem_t mtype = MSM_PMEM_MAX;
#elif defined(CONFIG_MACH_MOT)
	long rc = 0;
	struct axidata axi_data;
	int mtype = MSM_PMEM_MAX;
#else
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	int pmem_type = MSM_PMEM_MAX;
#endif

	struct msm_pmem_region region[3];


	memset(&axi_data, 0, sizeof(axi_data));

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (cfgcmd_t->cmd_type == CMD_STATS_AXI_CFG)
		mtype = MSM_PMEM_AEC_AWB;
	else if (cfgcmd_t->cmd_type == CMD_STATS_AF_AXI_CFG)
		mtype = MSM_PMEM_AF;

	axi_data.bufnum1 =
		msm_pmem_region_lookup(&msm->sync.stats, mtype,
			&region[0], NUM_WB_EXP_STAT_OUTPUT_BUFFERS);

	axi_data.region = &region[0];
	/* send the AEC/AWB STATS configuration command to driver */
	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(cfgcmd_t, &axi_data);
#else

       switch (cfgcmd->cmd_type) {
	case CMD_STATS_AXI_CFG:
		pmem_type = MSM_PMEM_AEC_AWB;
		break;
	case CMD_STATS_AF_AXI_CFG:
		pmem_type = MSM_PMEM_AF;
		break;
	case CMD_GENERAL:
		data = NULL;
		break;
	default:
	pr_err("%s: unknown command type %d\n",
		__FUNCTION__, cfgcmd->cmd_type);
		return -EINVAL;
	}
	if (cfgcmd->cmd_type != CMD_GENERAL) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats, pmem_type,
			&region[0], NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
			__FUNCTION__, __LINE__);
			return -EINVAL;
		}
	axi_data.region = &region[0];
	}
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, &axi_data);

#endif


	return rc;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_put_stats_buffer(void __user *arg,
	struct msm_device_t *msm)
{
	long rc = 0;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd_t;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf)))
		return -EFAULT;

	CVBS("msm_put_stats_buffer\n");
	msm_pmem_stats_vtop_lookup(buf.buffer,
		buf.fd, &pphy, msm);

	if (pphy != 0) {

		if (buf.type == STAT_AEAW)
			cfgcmd_t.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd_t.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else {
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd_t.value    = (void *)&buf;

		if (msm->vfefn.vfe_config)
			rc = msm->vfefn.vfe_config(&cfgcmd_t, &pphy);
	} else
		rc = -EFAULT;

put_done:
	return rc;
}
#else
static int msm_put_stats_buffer(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	CDBG("msm_put_stats_buffer\n");
	pphy = msm_pmem_stats_vtop_lookup(sync, buf.buffer, buf.fd);

	if (pphy != 0) {
		if (buf.type == STAT_AEAW)
			cfgcmd.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else {
			pr_err("%s: invalid buf type %d\n",
				__FUNCTION__,
				buf.type);
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd.value = (void *)&buf;

		if (sync->vfefn.vfe_config) {
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
			if (rc < 0)
				pr_err("msm_put_stats_buffer: "\
					"vfe_config err %d\n", rc);
		} else
			pr_err("msm_put_stats_buffer: vfe_config is NULL\n");
	} else {
		pr_err("msm_put_stats_buffer: NULL physical address\n");
		rc = -EINVAL;
	}

put_done:
	return rc;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_axi_config(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_axi_config(struct msm_sync *sync, void __user *arg)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	long rc = 0;
	struct msm_vfe_cfg_cmd cfgcmd_t;
#else
	struct msm_vfe_cfg_cmd cfgcmd;
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (copy_from_user(&cfgcmd_t, arg, sizeof(cfgcmd_t)))
		return -EFAULT;
#else
	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	switch (cfgcmd_t.cmd_type) {
#else
	switch (cfgcmd.cmd_type) {
#endif
	case CMD_AXI_CFG_OUT1:
	case CMD_AXI_CFG_OUT2:
	case CMD_AXI_CFG_SNAP_O1_AND_O2:
	case CMD_RAW_PICT_AXI_CFG:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		return msm_frame_axi_cfg(&cfgcmd_t, msm);
#else
		return msm_frame_axi_cfg(sync, &cfgcmd);
#endif

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		return msm_stats_axi_cfg(&cfgcmd_t, msm);
#else
		return msm_stats_axi_cfg(sync, &cfgcmd);
#endif

	default:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			return -EFAULT;
#else
		pr_err("%s: unknown command type %d\n",
			__FUNCTION__,
				cfgcmd.cmd_type);
			return -EINVAL;
#endif
	}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	return rc;
#else
	return 0;
#endif
}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int msm_camera_pict_pending(struct msm_device_t *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd_t *qcmd = NULL;

	spin_lock_irqsave(&msm->sync.pict_frame_q_lock,
		flags);

	if (!list_empty(&msm->sync.pict_frame_q)) {

		qcmd =
			list_first_entry(&msm->sync.pict_frame_q,
				struct msm_queue_cmd_t, list);

		if (qcmd) {
			if (qcmd->type  == MSM_CAM_Q_VFE_MSG)
				yes = 1;
		}
	}
	spin_unlock_irqrestore(&msm->sync.pict_frame_q_lock, flags);

	CVBS("msm_camera_pict_pending, yes = %d\n", yes);
	return yes;
}
#else

static int __msm_get_pic(struct msm_sync *sync, struct msm_ctrl_cmd *ctrl)
{
	unsigned long flags;
	int rc = 0;
	int tm;

	struct msm_queue_cmd *qcmd = NULL;

	tm = (int)ctrl->timeout_ms;

	rc = wait_event_interruptible_timeout(
			sync->pict_frame_wait,
			!list_empty_careful(&sync->pict_frame_q),
			msecs_to_jiffies(tm));
	if (list_empty_careful(&sync->pict_frame_q)) {
		if (rc == 0)
			return -ETIMEDOUT;
		if (rc < 0) {
			pr_err("msm_camera_get_picture, rc = %d\n", rc);
			return rc;
		}
	}

	spin_lock_irqsave(&sync->pict_frame_q_lock, flags);
	BUG_ON(list_empty(&sync->pict_frame_q));
	qcmd = list_first_entry(&sync->pict_frame_q,
			struct msm_queue_cmd, list);
	list_del_init(&qcmd->list);
	spin_unlock_irqrestore(&sync->pict_frame_q_lock, flags);

	if (qcmd->command != NULL) {
		struct msm_ctrl_cmd *q =
			(struct msm_ctrl_cmd *)qcmd->command;
		ctrl->type = q->type;
		ctrl->status = q->status;
	} else {
		ctrl->type = -1;
		ctrl->status = -1;
	}

	kfree(qcmd);
	return rc;
}

#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_get_pict_proc(struct msm_ctrl_cmd *ctrl,
	struct msm_device_t *msm)
{
	unsigned long flags;
	long rc = 0;
	int tm;
	struct msm_queue_cmd_t *qcmd = NULL;

	tm = (int)ctrl->timeout_ms;
	if (tm > 0) {
		rc =
			wait_event_timeout(
				msm->sync.pict_frame_wait,
				msm_camera_pict_pending(msm),
				msecs_to_jiffies(tm));

		if (rc == 0) {
			CDBG("msm_camera_get_picture, tm\n");
			return -ETIMEDOUT;
		}
	} else
		rc = wait_event_interruptible(
					msm->sync.pict_frame_wait,
					msm_camera_pict_pending(msm));

	if (rc < 0) {
		CDBG("msm_camera_get_picture, rc = %ld\n", rc);
		return -ERESTARTSYS;
	}
	spin_lock_irqsave(&msm->sync.pict_frame_q_lock, flags);
	if (!list_empty(&msm->sync.pict_frame_q)) {
		qcmd = list_first_entry(&msm->sync.pict_frame_q,
			struct msm_queue_cmd_t, list);
		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.pict_frame_q_lock, flags);
	if (qcmd->command != NULL) {
		ctrl->type =
		((struct msm_ctrl_cmd *)(qcmd->command))->type;

		ctrl->status =
		((struct msm_ctrl_cmd *)(qcmd->command))->status;

		kfree(qcmd->command);
	} else {
		ctrl->type = 0xFFFF;
		ctrl->status = 0xFFFF;
	}

	kfree(qcmd);

	return rc;
}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_get_pic(void __user *arg,
	struct msm_device_t *msm)
#else
static int msm_get_pic(struct msm_sync *sync, void __user *arg)
#endif
{
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_ctrl_cmd ctrlcmd_t;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd)))
		return -EFAULT;
#else
	struct msm_ctrl_cmd ctrlcmd_t;
	int rc;
	if (copy_from_user(&ctrlcmd_t,
				arg,
			sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	if (msm_get_pict_proc(&ctrlcmd_t, msm) < 0)
		return -EFAULT;
	if (msm->croplen) {
		if (ctrlcmd_t.length < msm->croplen)
		return -EINVAL;
#else
	rc = __msm_get_pic(sync, &ctrlcmd_t);
	if (rc < 0)
		return rc;

	if (sync->croplen) {
	if (ctrlcmd_t.length < sync->croplen) {
		pr_err("msm_get_pic: invalid len %d\n",
		ctrlcmd_t.length);
			return -EINVAL;
	}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		if (copy_to_user(ctrlcmd_t.value,
				msm->cropinfo,
				msm->croplen))
			return -EINVAL;
	}


	if (copy_to_user((void *)arg,
		&ctrlcmd_t,
		sizeof(struct msm_ctrl_cmd)))
		return -EFAULT;

#else
	if (copy_to_user(ctrlcmd_t.value,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			return -EFAULT;
		}
	}

	if (copy_to_user((void *)arg,
		&ctrlcmd_t,
		sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}

#endif
	return 0;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_set_crop(void __user *arg,
	struct msm_device_t *msm)
{
	struct crop_info crop;

	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info)))
		return -EFAULT;

	if (!msm->croplen) {
		msm->cropinfo = kmalloc(crop.len, GFP_KERNEL);

		if (!msm->cropinfo)
			return -ENOMEM;
	} else if (msm->croplen < crop.len)
		return -EINVAL;

	if (copy_from_user(msm->cropinfo,
				crop.info,
				crop.len)) {
		kfree(msm->cropinfo);
		return -EFAULT;
	}

	msm->croplen = crop.len;

	return 0;
}
#else

static int msm_set_crop(struct msm_sync *sync, void __user *arg)
{
	struct crop_info crop;

	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (!sync->croplen) {
		sync->cropinfo = kmalloc(crop.len, GFP_KERNEL);
		if (!sync->cropinfo)
			return -ENOMEM;
	} else if (sync->croplen < crop.len)
		return -EINVAL;

	if (copy_from_user(sync->cropinfo,
				crop.info,
				crop.len)) {
		ERR_COPY_FROM_USER();
		kfree(sync->cropinfo);
		return -EFAULT;
	}

	sync->croplen = crop.len;

	return 0;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_pict_pp_done(void __user *arg,
	struct msm_device_t *msm)
{
	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_ctrl_cmd *ctrlcmd = NULL;
	struct msm_queue_cmd_t *qcmd = NULL;
	unsigned long flags;
	long rc = 0;

	uint32_t pp_en;
	mutex_lock(&msm->pict_pp_lock);
	pp_en = msm->pict_pp;
	mutex_unlock(&msm->pict_pp_lock);
	CDBG("%s: %d is done\n", __func__, pp_en);

	if (!pp_en)
		return -EINVAL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))) {
		rc = -EFAULT;
		goto pp_done;
	}

	qcmd =
		kmalloc(sizeof(struct msm_queue_cmd_t),
						GFP_ATOMIC);
	if (!qcmd) {
		rc = -ENOMEM;
		goto pp_fail;
	}

	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!ctrlcmd) {
		rc = -ENOMEM;
		goto pp_done;
	}

	ctrlcmd->type = ctrlcmd_t.type;
	ctrlcmd->status = ctrlcmd_t.status;

pp_done:
	qcmd->type = MSM_CAM_Q_VFE_MSG;
	qcmd->command = ctrlcmd;

	spin_lock_irqsave(&msm->sync.pict_frame_q_lock, flags);
	list_add_tail(&qcmd->list, &msm->sync.pict_frame_q);
	spin_unlock_irqrestore(&msm->sync.pict_frame_q_lock, flags);
	wake_up(&msm->sync.pict_frame_wait);

pp_fail:
	return rc;
}
#else
static int msm_pict_pp_done(struct msm_sync *sync, void __user *arg)
{
	struct msm_ctrl_cmd udata;
	struct msm_ctrl_cmd *ctrlcmd = NULL;
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_vfe_phy_info *fphy;
	struct msm_frame frame;
	unsigned long pphy;

	unsigned long flags;
	int rc = 0;

	if (!sync->pict_pp)
		return -EINVAL;

	if (copy_from_user(&udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto pp_fail;
	}

	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!ctrlcmd) {
		rc = -ENOMEM;
		goto pp_fail;
	}


	qcmd = kmalloc(sizeof(struct msm_queue_cmd) +
			sizeof(struct msm_ctrl_cmd),
			GFP_KERNEL);
	if (!qcmd) {
		rc = -ENOMEM;
		goto pp_fail;
	}

	if (sync->pict_pp & PP_PREV) {
		fphy = kzalloc(sizeof(struct msm_vfe_phy_info),
			GFP_ATOMIC);
		if (!fphy) {
			CDBG("%s: cannot allocate buffer at line=%d\n",
				__func__, __LINE__);
			kfree(ctrlcmd);
			kfree(qcmd);
			rc = -ENOMEM;
			goto pp_fail;
		}
		if (udata.length) {
			ctrlcmd->value = kmalloc(udata.length, GFP_ATOMIC);
			if (!ctrlcmd->value) {
				CDBG("%s: cannot allocate buffer at line=%d\n",
					__func__, __LINE__);
				rc = -ENOMEM;
				goto pp_fail;
			}
			if (copy_from_user(ctrlcmd->value,
						udata.value,
						udata.length)) {
				CDBG("%s: copy_from_user failed at line=%d\n",
					 __func__, __LINE__);
				kfree(ctrlcmd);
				kfree(ctrlcmd->value);
				rc = -EFAULT;
				goto pp_fail;
			}
		}
		memcpy(&frame, ctrlcmd->value, udata.length);

		CDBG("%s: buffer=%ld, y_off=%u, cbcr_off=%u, fd=%d\n",
			__func__,
			frame.buffer, frame.y_off, frame.cbcr_off, frame.fd);

		pphy = msm_pmem_frame_vtop_lookup(sync, frame.buffer,
			frame.y_off, frame.cbcr_off, frame.fd);

		CDBG("%s: vaddr = 0x%lx, pphy = 0x%lx\n", __func__,
			frame.buffer, pphy);

		if (pphy != 0) {
			fphy->y_phy = pphy + frame.y_off;
			fphy->cbcr_phy = pphy + frame.cbcr_off;
			fphy->sbuf_phy = 0;
		}
		CDBG("%s:vaddr = 0x%lx, y_phy = 0x%x cbcr_phy=0x%x \n",
			__func__, frame.buffer, fphy->y_phy, fphy->cbcr_phy);

		qcmd->type    = MSM_CAM_Q_VFE_MSG;
		qcmd->command = fphy;
		CDBG("%s: phy_y= 0x%x, phy_cbcr= 0x%x\n", __func__,
			fphy->y_phy, fphy->cbcr_phy);
		spin_lock_irqsave(&sync->prev_frame_q_lock,
			flags);
		list_add_tail(&qcmd->list,
			&sync->prev_frame_q);
		spin_unlock_irqrestore(&sync->prev_frame_q_lock,
			flags);
		wake_up(&sync->prev_frame_wait);
		CDBG("%s: waked up frame thread\n", __func__);
		kfree(ctrlcmd);
		kfree(ctrlcmd->value);

	} else if (sync->pict_pp & PP_SNAP) {
		ctrlcmd->type = udata.type;
		ctrlcmd->status = udata.status;

		qcmd->type = MSM_CAM_Q_VFE_MSG;
		qcmd->command = ctrlcmd = (struct msm_ctrl_cmd *)(qcmd + 1);
		memset(ctrlcmd, 0, sizeof(struct msm_ctrl_cmd));
		ctrlcmd->type = udata.type;
		ctrlcmd->status = udata.status;

		spin_lock_irqsave(&sync->pict_frame_q_lock, flags);
		list_add_tail(&qcmd->list, &sync->pict_frame_q);
		spin_unlock_irqrestore(&sync->pict_frame_q_lock, flags);
		wake_up(&sync->pict_frame_wait);
	}

pp_fail:
	return rc;
}

#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int msm_af_status_pending(struct msm_device_t *msm)
{
	int rc;
	unsigned long flags;
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	rc = msm->sync.af_flag;
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	return rc;
}
static long msm_af_control(void __user *arg,
	struct msm_device_t *msm)
{
	unsigned long flags;
	int timeout;
	long rc = 0;
	if (copy_from_user(&msm->sync.af_status,
			arg, sizeof(struct msm_ctrl_cmd))) {
		rc = -EFAULT;
		goto end;
	}
	timeout = (int)msm->sync.af_status.timeout_ms;
	CVBS("msm_af_control, timeout = %d\n", timeout);
	if (timeout > 0) {
		rc = wait_event_timeout(msm->sync.af_status_wait,
			msm_af_status_pending(msm),
			msecs_to_jiffies(timeout));
		CVBS("msm_af_control: rc = %ld\n", rc);
		if (rc == 0) {
			CVBS("msm_af_control: timed out\n");
			rc = -ETIMEDOUT;
			goto end;
		}
	} else
		rc = wait_event_interruptible(msm->sync.af_status_wait,
			msm_af_status_pending(msm));
	if (rc < 0) {
		rc = -EAGAIN;
		goto end;
	}
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	if (msm->sync.af_flag < 0) {
		msm->sync.af_status.type = 0xFFFF;
		msm->sync.af_status.status = 0xFFFF;
	}
	msm->sync.af_flag = 0;
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	if (copy_to_user((void *)arg,
			&msm->sync.af_status,
			sizeof(struct msm_ctrl_cmd))) {
		CDBG("msm_af_control: copy_to_user ctrlcmd failed!\n");
		rc = -EFAULT;
	}

end:
    CDBG("msm_af_control: end rc = %ld\n", rc);

	return rc;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_af_control_done(void __user *arg,
	struct msm_device_t *msm)
{
	unsigned long flags;
	long rc = 0;
	rc = copy_from_user(&msm->sync.af_status,
		arg, sizeof(struct msm_ctrl_cmd));
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	msm->sync.af_flag = (rc == 0 ? 1 : -1);
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	wake_up(&msm->sync.af_status_wait);
	return rc;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_ioctl(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct msm_device_t *pmsm = filep->private_data;

	CDBG("msm_ioctl cmd = %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		return msm_get_sensor_info(argp, pmsm);

	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(argp, pmsm);

	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(argp, pmsm);

	case MSM_CAM_IOCTL_CTRL_COMMAND:
		/* Coming from control thread, may need to wait for
		* command status */
		return msm_control(argp, pmsm);

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		return msm_config_vfe(argp, pmsm);

	case MSM_CAM_IOCTL_GET_STATS:
		/* Coming from config thread wait
		* for vfe statistics and control requests */
		return msm_get_stats(argp, pmsm);

	case MSM_CAM_IOCTL_GETFRAME:
		/* Coming from frame thread to get frame
		* after SELECT is done */
		return msm_get_frame(argp, pmsm);

	case MSM_CAM_IOCTL_ENABLE_VFE:
		/* This request comes from control thread:
		* enable either QCAMTASK or VFETASK */
		return msm_enable_vfe(argp, pmsm);

	case MSM_CAM_IOCTL_CTRL_CMD_DONE:
		/* Config thread notifies the result of contrl command */
		return msm_ctrl_cmd_done(argp, pmsm);

	case MSM_CAM_IOCTL_VFE_APPS_RESET:
		msm_camio_vfe_blk_reset();
		return 0;

	case MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER:
		return msm_put_frame_buffer(argp, pmsm);

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		return msm_put_stats_buffer(argp, pmsm);

	case MSM_CAM_IOCTL_AXI_CONFIG:
		return msm_axi_config(argp, pmsm);

	case MSM_CAM_IOCTL_GET_PICTURE:
		return msm_get_pic(argp, pmsm);

	case MSM_CAM_IOCTL_SET_CROP:
		return msm_set_crop(argp, pmsm);

	case MSM_CAM_IOCTL_PICT_PP: {
		uint32_t pp;
		if (copy_from_user(&pp, argp, sizeof(pp)))
			return -EFAULT;
		mutex_lock(&pmsm->pict_pp_lock);
		pmsm->pict_pp = pp;
		mutex_unlock(&pmsm->pict_pp_lock);
		CDBG("%s: case: MSM_CAM_IOCTL_PICT_PP: pp = %d \n", __func__, pp);
		return 0;
	}

	case MSM_CAM_IOCTL_PICT_PP_DONE:
		return msm_pict_pp_done(argp, pmsm);

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		return pmsm->sctrl.s_config(argp);

	case MSM_CAM_IOCTL_FLASH_LED_CFG: {
		unsigned int led_state;
		if (copy_from_user(&led_state, argp, sizeof(led_state)))
			return -EFAULT;
		return  msm_camera_flash_set_led_state(led_state);
	}
#ifdef CONFIG_MACH_MOT
    case MSM_CAM_IOCTL_DISABLE_LENS_MOVE:
        printk("CAM_TCMD: lens move disabling.");
        disable_lens_move = 1;
        return 0;

    case MSM_CAM_IOCTL_ENABLE_LENS_MOVE:
        printk("CAM_TCMD: lens move enabling.");
        disable_lens_move = 0;
        return 0;
#endif

	case MSM_CAM_IOCTL_AF_CTRL:
		return msm_af_control(argp, pmsm);

	case MSM_CAM_IOCTL_AF_CTRL_DONE:
		return msm_af_control_done(argp, pmsm);

	default:
		break;
	}

	return -EINVAL;
}
#else
static long msm_ioctl_common(struct msm_device *pmsm,
		unsigned int cmd,
		void __user *argp)
{
	CDBG("msm_ioctl_common\n");
	switch (cmd) {
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(pmsm->sync, argp);
	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(pmsm->sync, argp);
	default:
		return -EINVAL;
	}
}

static long msm_ioctl_config(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_device *pmsm = filep->private_data;

	CDBG("msm_ioctl_config cmd = %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		rc = msm_config_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_GET_STATS:
		/* Coming from config thread wait
		 * for vfe statistics and control requests */
		rc = msm_get_stats(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_ENABLE_VFE:
		/* This request comes from control thread:
		 * enable either QCAMTASK or VFETASK */
		rc = msm_enable_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_DISABLE_VFE:
		/* This request comes from control thread:
		 * disable either QCAMTASK or VFETASK */
		rc = msm_disable_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_VFE_APPS_RESET:
		msm_camio_vfe_blk_reset();
		rc = 0;
		break;

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		rc = msm_put_stats_buffer(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_AXI_CONFIG:
		rc = msm_axi_config(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SET_CROP:
		rc = msm_set_crop(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_PICT_PP: {
		uint8_t enable;
		if (copy_from_user(&enable, argp, sizeof(enable))) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else {
			pmsm->sync->pict_pp = enable;
			rc = 0;
		}
		break;
	}

	case MSM_CAM_IOCTL_PICT_PP_DONE:
		rc = msm_pict_pp_done(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		rc = pmsm->sync->sctrl.s_config(argp);
		break;

	case MSM_CAM_IOCTL_FLASH_LED_CFG: {
		uint32_t led_state;
		if (copy_from_user(&led_state, argp, sizeof(led_state))) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else
			rc = msm_camera_flash_set_led_state(led_state);
		break;
	}

	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	CDBG("msm_ioctl_config cmd = %d DONE\n", _IOC_NR(cmd));
	return rc;
}

static int msm_unblock_poll_frame(struct msm_sync *);

static long msm_ioctl_frame(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_device *pmsm = filep->private_data;


	switch (cmd) {
	case MSM_CAM_IOCTL_GETFRAME:
		/* Coming from frame thread to get frame
		 * after SELECT is done */
		rc = msm_get_frame(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER:
		rc = msm_put_frame_buffer(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME:
		rc = msm_unblock_poll_frame(pmsm->sync);
		break;
	default:
		break;
	}

	return rc;
}


static long msm_ioctl_control(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_device *pmsm = ctrl_pmsm->pmsm;

	switch (cmd) {
	case MSM_CAM_IOCTL_CTRL_COMMAND:
		/* Coming from control thread, may need to wait for
		 * command status */
		rc = msm_control(ctrl_pmsm, 1, argp);
		break;
	case MSM_CAM_IOCTL_CTRL_COMMAND_2:
		/* Sends a message, returns immediately */
		rc = msm_control(ctrl_pmsm, 0, argp);
		break;
	case MSM_CAM_IOCTL_CTRL_CMD_DONE:
		/* Config thread calls the control thread to notify it
		 * of the result of a MSM_CAM_IOCTL_CTRL_COMMAND.
		 */
		rc = msm_ctrl_cmd_done(ctrl_pmsm, argp);
		break;
	case MSM_CAM_IOCTL_GET_PICTURE:
		rc = msm_get_pic(pmsm->sync, argp);
		break;
	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	return rc;
}

#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int msm_frame_pending(struct msm_device_t *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd_t *qcmd = NULL;

	spin_lock_irqsave(&msm->sync.prev_frame_q_lock, flags);

	if (!list_empty(&msm->sync.prev_frame_q)) {

		qcmd = list_first_entry(&msm->sync.prev_frame_q,
			struct msm_queue_cmd_t, list);

		if (!qcmd)
			yes = 0;
		else {
			yes = 1;
			CVBS("msm_frame_pending: yes = %d\n",
				yes);
		}
	}

	spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock, flags);

	CVBS("msm_frame_pending, yes = %d\n", yes);
	return yes;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int msm_release(struct inode *node, struct file *filep)
{
	struct msm_pmem_region *region;
	struct hlist_node *hnode;
	struct hlist_node *n;
	struct msm_queue_cmd_t *qcmd = NULL;
	unsigned long flags;
	struct msm_device_t *pmsm = filep->private_data;

	mutex_lock(&pmsm->msm_lock);
	pmsm->opencnt -= 1;
	mutex_unlock(&pmsm->msm_lock);

	if (!pmsm->opencnt) {
		/* need to clean up
		 * system resource */
		if (pmsm->vfefn.vfe_release)
			pmsm->vfefn.vfe_release(pmsm->pdev);

		if (pmsm->croplen) {
			kfree(pmsm->cropinfo);
			pmsm->croplen = 0;
		}

		hlist_for_each_entry_safe(region, hnode, n,
			&pmsm->sync.frame, list) {

			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		hlist_for_each_entry_safe(region, hnode, n,
			&pmsm->sync.stats, list) {

			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		while (msm_ctrl_stats_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.ctrl_status_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.ctrl_status_queue,
				struct msm_queue_cmd_t, list);
			spin_unlock_irqrestore(&pmsm->sync.ctrl_status_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				if (qcmd->type == MSM_CAM_Q_VFE_MSG)
					kfree(((struct msm_vfe_resp_t *)
						(qcmd->command))->evt_msg.data);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		while (msm_stats_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.msg_event_queue_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.msg_event_queue,
				struct msm_queue_cmd_t, list);
			spin_unlock_irqrestore(&pmsm->sync.msg_event_queue_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		while (msm_camera_pict_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.pict_frame_q_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.pict_frame_q,
				struct msm_queue_cmd_t, list);
			spin_unlock_irqrestore(&pmsm->sync.pict_frame_q_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		while (msm_frame_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.prev_frame_q_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.prev_frame_q,
				struct msm_queue_cmd_t, list);
			spin_unlock_irqrestore(&pmsm->sync.prev_frame_q_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		pmsm->sctrl.s_release();

		CDBG("msm_release completed!\n");
	}

	return 0;
}

static ssize_t msm_read(struct file *filep, char __user *arg,
	size_t size, loff_t *loff)
{
	return 0;
}

static ssize_t msm_write(struct file *filep, const char __user *arg,
	size_t size, loff_t *loff)
{
	return 0;
}

unsigned int msm_poll(struct file *filep,
	struct poll_table_struct *pll_table)
{
	struct msm_device_t *pmsm = filep->private_data;
	struct msm_queue_cmd_t *qcmd = NULL;
	unsigned long flags;

	while (msm_camera_pict_pending(pmsm)) {
		spin_lock_irqsave(&pmsm->sync.pict_frame_q_lock,
			flags);
		qcmd = list_first_entry(&pmsm->sync.pict_frame_q,
			struct msm_queue_cmd_t, list);
		spin_unlock_irqrestore(&pmsm->sync.pict_frame_q_lock,
			flags);

		if (qcmd) {
			list_del(&qcmd->list);
			kfree(qcmd->command);
			kfree(qcmd);
		}
	};

	poll_wait(filep, &pmsm->sync.prev_frame_wait, pll_table);

	if (msm_frame_pending(pmsm))
		/* frame ready */
		return POLLIN | POLLRDNORM;

	return 0;
}
#else
static int __msm_release(struct msm_sync *sync)
{
	struct msm_pmem_region *region;
	struct hlist_node *hnode;
	struct hlist_node *n;

	mutex_lock(&sync->lock);
	if (sync->opencnt)
		sync->opencnt--;

	if (!sync->opencnt) {
		/* need to clean up system resource */
		if (sync->vfefn.vfe_release)
			sync->vfefn.vfe_release(sync->pdev);

		if (sync->cropinfo) {
		kfree(sync->cropinfo);
		sync->cropinfo = NULL;
		sync->croplen = 0;
		}

		hlist_for_each_entry_safe(region, hnode, n,
				&sync->frame, list) {
			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		hlist_for_each_entry_safe(region, hnode, n,
				&sync->stats, list) {
			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		MSM_DRAIN_QUEUE(sync, msg_event_q);
		MSM_DRAIN_QUEUE(sync, prev_frame_q);
		MSM_DRAIN_QUEUE(sync, pict_frame_q);

		sync->sctrl.s_release();
		wake_unlock(&sync->wake_lock);

		sync->apps_id = NULL;
		CDBG("msm_release completed!\n");
	}
	mutex_unlock(&sync->lock);

	return 0;
}

static int msm_release_config(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_device *pmsm = filep->private_data;
	CDBG("msm_camera: RELEASE %s\n", filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	atomic_set(&pmsm->opened, 0);
	return rc;
}

static int msm_release_control(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_device *pmsm = ctrl_pmsm->pmsm;
	CDBG("msm_camera: RELEASE %s\n", filep->f_path.dentry->d_name.name);
	g_v4l2_opencnt--;
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		MSM_DRAIN_QUEUE(&ctrl_pmsm->ctrl_q, ctrl_status_q);
		MSM_DRAIN_QUEUE(pmsm->sync, pict_frame_q);
	}
	kfree(ctrl_pmsm);
	return rc;
}

static int msm_release_frame(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_device *pmsm = filep->private_data;
	CDBG("msm_camera: RELEASE %s\n", filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		MSM_DRAIN_QUEUE(pmsm->sync, prev_frame_q);
		atomic_set(&pmsm->opened, 0);
	}
	return rc;
}

static int msm_unblock_poll_frame(struct msm_sync *sync)
{
	unsigned long flags;
	CDBG("msm_unblock_poll_frame\n");
	spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
	sync->unblock_poll_frame = 1;
	wake_up(&sync->prev_frame_wait);
	spin_unlock_irqrestore(&sync->prev_frame_q_lock, flags);
	return 0;
}

static unsigned int __msm_poll_frame(struct msm_sync *sync,
		struct file *filep,
		struct poll_table_struct *pll_table)
{
	int rc = 0;
	unsigned long flags;

	poll_wait(filep, &sync->prev_frame_wait, pll_table);

	spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
	if (!list_empty_careful(&sync->prev_frame_q))
		/* frame ready */
		rc = POLLIN | POLLRDNORM;
	if (sync->unblock_poll_frame) {
		CDBG("%s: sync->unblock_poll_frame is true\n", __func__);
		rc |= POLLPRI;
		sync->unblock_poll_frame = 0;
	}
	spin_unlock_irqrestore(&sync->prev_frame_q_lock, flags);

	return rc;
}

static unsigned int msm_poll_frame(struct file *filep,
	struct poll_table_struct *pll_table)
{
	struct msm_device *pmsm = filep->private_data;
	return __msm_poll_frame(pmsm->sync, filep, pll_table);
}

/*
 * This function executes in interrupt context.
 */
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static void msm_vfe_sync(struct msm_vfe_resp_t *vdata,
	 enum msm_queut_t qtype, void *syncdata)
{
	struct msm_queue_cmd_t *qcmd = NULL;
	struct msm_queue_cmd_t *qcmd_frame = NULL;
	struct msm_vfe_phy_info *fphy;
	uint32_t pp;

	unsigned long flags;
	struct msm_device_t *msm =
		(struct msm_device_t *)syncdata;

	if (!msm)
		return;

	qcmd = kmalloc(sizeof(struct msm_queue_cmd_t),
					GFP_ATOMIC);
	if (!qcmd) {
		CDBG("evt_msg: cannot allocate buffer\n");
		goto mem_fail1;
	}

	if (qtype == MSM_CAM_Q_VFE_EVT) {
		qcmd->type    = MSM_CAM_Q_VFE_EVT;
	} else if (qtype == MSM_CAM_Q_VFE_MSG) {

		qcmd->type = MSM_CAM_Q_VFE_MSG;

		if (vdata->type == VFE_MSG_OUTPUT1 ||
		    vdata->type == VFE_MSG_OUTPUT2) {

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
			mutex_lock(&msm->pict_pp_lock);
#endif
			pp = msm->pict_pp;
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
			mutex_unlock(&msm->pict_pp_lock);
#endif

			if (pp & PP_PREV)
				goto sync_done;

			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd_t),
					GFP_ATOMIC);
			if (!qcmd_frame)
				goto mem_fail2;

			fphy = kmalloc(sizeof(struct msm_vfe_phy_info),
				GFP_ATOMIC);
			if (!fphy)
				goto mem_fail3;

			*fphy = vdata->phy;

			qcmd_frame->type    = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = fphy;

			CVBS("qcmd_frame= %p phy_y= 0x%x, phy_cbcr= 0x%x\n",
			      qcmd_frame, fphy->y_phy, fphy->cbcr_phy);

			spin_lock_irqsave(&msm->sync.prev_frame_q_lock,
				flags);

			list_add_tail(&qcmd_frame->list,
				&msm->sync.prev_frame_q);

			spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock,
				flags);

			wake_up(&msm->sync.prev_frame_wait);
			CVBS("waked up frame thread\n");

		} else if (vdata->type == VFE_MSG_SNAPSHOT) {

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
			mutex_lock(&msm->pict_pp_lock);
#endif
			pp = msm->pict_pp;
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
			mutex_unlock(&msm->pict_pp_lock);
#endif
            if ((pp & PP_SNAP) || (pp & PP_RAW_SNAP))
                goto sync_done;

			CDBG("SNAPSHOT pp = %d\n", pp);
            qcmd_frame =
                kmalloc(sizeof(struct msm_queue_cmd_t),
                        GFP_ATOMIC);
            if (!qcmd_frame)
                goto mem_fail2;

            qcmd_frame->type    = MSM_CAM_Q_VFE_MSG;
            qcmd_frame->command = NULL;

            spin_lock_irqsave(&msm->sync.pict_frame_q_lock,
                    flags);

            list_add_tail(&qcmd_frame->list,
                    &msm->sync.pict_frame_q);

            spin_unlock_irqrestore(
                    &msm->sync.pict_frame_q_lock, flags);
            wake_up(&msm->sync.pict_frame_wait);
        }
	}

sync_done:
	qcmd->command = (void *)vdata;
	CVBS("vdata->type = %d\n", vdata->type);

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock,
		flags);
	list_add_tail(&qcmd->list, &msm->sync.msg_event_queue);
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock,
		flags);
	wake_up(&msm->sync.msg_event_wait);
	CVBS("waked up config thread\n");

	return;

mem_fail3:
	kfree(qcmd_frame);

mem_fail2:
	kfree(qcmd);

mem_fail1:
	if (qtype == MSM_CAMERA_MSG &&
			vdata->evt_msg.len > 0)
		kfree(vdata->evt_msg.data);

	kfree(vdata);
	return;
}

static struct msm_vfe_resp msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
};
#else
static void *msm_vfe_sync_alloc(int size,
			void *syncdata __attribute__((unused)))
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) + size, GFP_ATOMIC);
	return qcmd ? qcmd + 1 : NULL;
}

/*
 * This function executes in interrupt context.
 */

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
		enum msm_queue qtype, void *syncdata)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_queue_cmd *qcmd_frame = NULL;
	struct msm_vfe_phy_info *fphy;

	unsigned long flags;
	struct msm_sync *sync = (struct msm_sync *)syncdata;
	if (!sync) {
		pr_err("msm_camera: no context in dsp callback.\n");
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;

	if (qtype == MSM_CAM_Q_VFE_MSG) {
		switch(vdata->type) {
		case VFE_MSG_OUTPUT1:
		case VFE_MSG_OUTPUT2:
			if (sync->pict_pp & PP_PREV) {
				CDBG("%s: PP_PREV is in progess.\n",
					__func__);
				CDBG("%s:phy_y=0x%x,phy_cbcr=0x%x\n",
					__func__, vdata->phy.y_phy,
				vdata->phy.cbcr_phy);
				sync->pp_sync_flag = 1;
				break;
			}

			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd) +
					sizeof(struct msm_vfe_phy_info),
					GFP_ATOMIC);
			if (!qcmd_frame)
				goto mem_fail;
			fphy = (struct msm_vfe_phy_info *)(qcmd_frame + 1);
			*fphy = vdata->phy;

			qcmd_frame->type = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = fphy;

			CDBG("qcmd_frame= 0x%x phy_y= 0x%x, phy_cbcr= 0x%x\n",
				(int) qcmd_frame, fphy->y_phy, fphy->cbcr_phy);

			spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
			list_add_tail(&qcmd_frame->list, &sync->prev_frame_q);
			wake_up(&sync->prev_frame_wait);
			spin_unlock_irqrestore(&sync->prev_frame_q_lock, flags);
			CDBG("woke up frame thread\n");
			break;
		case VFE_MSG_SNAPSHOT:
			if ((sync->pict_pp & PP_SNAP) ||
					(sync->pict_pp & PP_RAW_SNAP)) {
				CDBG("%s: sync_data, sync->pict_pp = %d\n",
					__func__, sync->pict_pp);
				break;
			}

			CDBG("snapshot pp = %d\n", sync->pict_pp);
			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd),
					GFP_ATOMIC);
			if (!qcmd_frame)
				goto mem_fail;
			qcmd_frame->type = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = NULL;
				spin_lock_irqsave(&sync->pict_frame_q_lock,
				flags);
			list_add_tail(&qcmd_frame->list, &sync->pict_frame_q);
			wake_up(&sync->pict_frame_wait);
			spin_unlock_irqrestore(&sync->pict_frame_q_lock, flags);
			CDBG("woke up picture thread\n");
			break;
		default:
			CDBG("%s: qtype = %d not handled\n",
				__func__, vdata->type);
			break;
		}
	}

	qcmd->command = (void *)vdata;
	CDBG("vdata->type = %d\n", vdata->type);

	spin_lock_irqsave(&sync->msg_event_q_lock, flags);
	list_add_tail(&qcmd->list, &sync->msg_event_q);
	wake_up(&sync->msg_event_wait);
	spin_unlock_irqrestore(&sync->msg_event_q_lock, flags);
	CDBG("woke up config thread\n");
	return;

mem_fail:
	kfree(qcmd);
}

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
	.vfe_alloc = msm_vfe_sync_alloc,
};
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_open_proc(struct msm_device_t *msm)
{
	long rc = 0;
	struct msm_camera_device_platform_data *pdata =
		msm->pdev->dev.platform_data;

	rc = msm_camvfe_check(msm);
	if (rc < 0)
		goto msm_open_proc_done;

	if (!pdata) {
		rc = -ENODEV;
		goto msm_open_proc_done;
	}

	mutex_lock(&msm->msm_lock);
	if (msm->opencnt > 5) {
		mutex_unlock(&msm->msm_lock);
		return -EFAULT;
	}
	msm->opencnt += 1;
	mutex_unlock(&msm->msm_lock);

	if (msm->opencnt == 1) {
		msm_camvfe_fn_init(&msm->vfefn);

		if (msm->vfefn.vfe_init) {

			rc = msm->vfefn.vfe_init(&msm_vfe_s,
				msm->pdev);
			if (rc < 0) {
				CDBG("vfe_init failed with %ld\n", rc);
				msm->opencnt -= 1;
				goto msm_open_proc_done;
			}
			rc = msm->sctrl.s_init(&pdata->sinfo[msm->sidx]);
			if (rc < 0) {
				CDBG("sensor init failed with %ld\n", rc);
                // Motorola addition to release vfe
                if (msm->vfefn.vfe_release)
                    msm->vfefn.vfe_release(msm->pdev);
				msm->opencnt -= 1;
				goto msm_open_proc_done;
			}
		} else {
			rc = -ENODEV;
			msm->opencnt -= 1;
			goto msm_open_proc_done;
		}

		mutex_lock(&msm->msm_sem);
		if (rc >= 0) {
			INIT_HLIST_HEAD(&msm->sync.frame);
			INIT_HLIST_HEAD(&msm->sync.stats);
		}
		mutex_unlock(&msm->msm_sem);

	} else if (msm->opencnt > 1)
		rc = 0;

msm_open_proc_done:
	return rc;
}

static int msm_open(struct inode *inode, struct file *filep)
{
	struct msm_device_t *pmsm;
	int rc = 0;

	rc = nonseekable_open(inode, filep);
	if (rc < 0)
		goto cam_open_fail;

	pmsm = container_of(inode->i_cdev,
		struct msm_device_t, cdev);
	if (!pmsm) {
		rc = -ENODEV;
		goto cam_open_fail;
	}

	rc = msm_open_proc(pmsm);
	if (rc < 0)
		goto cam_open_done;

	filep->private_data = pmsm;

cam_open_fail:
cam_open_done:
	CDBG("msm_open() open: rc = %d\n", rc);
	return rc;
}

static struct file_operations msm_fops = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl,
	.release = msm_release,
	.read = msm_read,
	.write = msm_write,
	.poll = msm_poll,
};
#else
static int __msm_open(struct msm_sync *sync, const char *const apps_id)
{
	int rc = 0;

	mutex_lock(&sync->lock);
	if (sync->apps_id && strcmp(sync->apps_id, apps_id)
				&& (!strcmp(MSM_APPS_ID_V4L2, apps_id))) {
		pr_err("msm_camera(%s): sensor %s is already opened for %s\n",
			apps_id,
			sync->sdata->sensor_name,
			sync->apps_id);
		rc = -EBUSY;
		goto msm_open_done;
	}

	sync->apps_id = apps_id;

	if (!sync->opencnt) {
		wake_lock(&sync->wake_lock);

		msm_camvfe_fn_init(&sync->vfefn, sync);
		if (sync->vfefn.vfe_init) {
			rc = sync->vfefn.vfe_init(&msm_vfe_s,
				sync->pdev);
			if (rc < 0) {
				pr_err("vfe_init failed at %d\n", rc);
				goto msm_open_done;
			}
			rc = sync->sctrl.s_init(sync->sdata);
			if (rc < 0) {
				pr_err("sensor init failed: %d\n", rc);
				goto msm_open_done;
			}
		} else {
			pr_err("no sensor init func\n");
			rc = -ENODEV;
			goto msm_open_done;
		}

		if (rc >= 0) {
			INIT_HLIST_HEAD(&sync->frame);
			INIT_HLIST_HEAD(&sync->stats);
			sync->unblock_poll_frame = 0;
		}
	}
	sync->opencnt++;

msm_open_done:
	mutex_unlock(&sync->lock);
	return rc;
}

static int msm_open_common(struct inode *inode, struct file *filep,
			   int once)
{
	int rc;
	struct msm_device *pmsm =
		container_of(inode->i_cdev, struct msm_device, cdev);

	CDBG("msm_camera: open %s\n", filep->f_path.dentry->d_name.name);

	if (atomic_cmpxchg(&pmsm->opened, 0, 1) && once) {
		pr_err("msm_camera: %s is already opened.\n",
			filep->f_path.dentry->d_name.name);
		return -EBUSY;
	}

	rc = nonseekable_open(inode, filep);
	if (rc < 0) {
		pr_err("msm_open: nonseekable_open error %d\n", rc);
		return rc;
	}

	rc = __msm_open(pmsm->sync, MSM_APPS_ID_PROP);
	if (rc < 0)
		return rc;

	filep->private_data = pmsm;

	CDBG("msm_open() open: rc = %d\n", rc);
	return rc;
}

static int msm_open(struct inode *inode, struct file *filep)
{
	return msm_open_common(inode, filep, 1);
}

static int msm_open_control(struct inode *inode, struct file *filep)
{
	int rc;

	struct msm_control_device *ctrl_pmsm =
		kmalloc(sizeof(struct msm_control_device), GFP_KERNEL);
	if (!ctrl_pmsm)
		return -ENOMEM;

	rc = msm_open_common(inode, filep, 0);
	if (rc < 0)
		return rc;

	ctrl_pmsm->pmsm = filep->private_data;
	filep->private_data = ctrl_pmsm;
	spin_lock_init(&ctrl_pmsm->ctrl_q.ctrl_status_q_lock);
	INIT_LIST_HEAD(&ctrl_pmsm->ctrl_q.ctrl_status_q);
	init_waitqueue_head(&ctrl_pmsm->ctrl_q.ctrl_status_wait);
	if (!g_v4l2_opencnt)
		g_v4l2_control_device = ctrl_pmsm;

	g_v4l2_opencnt++;

	CDBG("msm_open() open: rc = %d\n", rc);
	return rc;
}

static int __msm_v4l2_control(struct msm_sync *sync,
		struct msm_ctrl_cmd *out)
{
	int rc = 0;

	struct msm_queue_cmd *qcmd = NULL, *rcmd = NULL;
	struct msm_ctrl_cmd *ctrl;
	struct msm_control_device_queue
			*v4l2_ctrl_q = &g_v4l2_control_device->ctrl_q;

	/* wake up config thread, 4 is for V4L2 application */
	qcmd = kmalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!qcmd) {
		pr_err("msm_control: cannot allocate buffer\n");
		rc = -ENOMEM;
		goto end;
	}
	qcmd->type = MSM_CAM_Q_V4L2_REQ;
	qcmd->command = out;

	if (out->type == V4L2_CAMERA_EXIT) {
		rcmd = __msm_control(sync, NULL, qcmd, out->timeout_ms);
		if (rcmd == NULL) {
			rc = PTR_ERR(rcmd);
			goto end;
		}
	}

	rcmd = __msm_control(sync, v4l2_ctrl_q, qcmd, out->timeout_ms);
	if (IS_ERR(rcmd)) {
		rc = PTR_ERR(rcmd);
		goto end;
	}

	ctrl = (struct msm_ctrl_cmd *)(rcmd->command);
	/* FIXME: we should just set out->length = ctrl->length; */
	BUG_ON(out->length < ctrl->length);
	memcpy(out->value, ctrl->value, ctrl->length);

end:

	if (rcmd) kfree(rcmd);
	CDBG("__msm_v4l2_control: end rc = %d\n", rc);
	return rc;
}

static const struct file_operations msm_fops_config = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl_config,
	.release = msm_release_config,
};

static const struct file_operations msm_fops_control = {
	.owner = THIS_MODULE,
	.open = msm_open_control,
	.unlocked_ioctl = msm_ioctl_control,
	.release = msm_release_control,
};

static const struct file_operations msm_fops_frame = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl_frame,
	.release = msm_release_frame,
	.poll = msm_poll_frame,
};
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_setup_cdevs(struct msm_device_t *msm,
	dev_t devno)
{
	int rc = -ENODEV;
	struct device *class_dev;
	char name[20];

	sprintf(name, "%s%d", "msm_camera", MINOR(devno));
	class_dev = device_create(msm_class, NULL, devno, "%s", name);
	if (IS_ERR(class_dev))
		goto setup_fail_return;

	spin_lock_init(&msm->sync.msg_event_queue_lock);
	INIT_LIST_HEAD(&msm->sync.msg_event_queue);
	init_waitqueue_head(&msm->sync.msg_event_wait);

	spin_lock_init(&msm->sync.prev_frame_q_lock);
	INIT_LIST_HEAD(&msm->sync.prev_frame_q);
	init_waitqueue_head(&msm->sync.prev_frame_wait);

	spin_lock_init(&msm->sync.pict_frame_q_lock);
	INIT_LIST_HEAD(&msm->sync.pict_frame_q);
	init_waitqueue_head(&msm->sync.pict_frame_wait);

	spin_lock_init(&msm->sync.ctrl_status_lock);
	INIT_LIST_HEAD(&msm->sync.ctrl_status_queue);
	init_waitqueue_head(&msm->sync.ctrl_status_wait);
	spin_lock_init(&msm->sync.af_status_lock);
	msm->sync.af_flag = 0;
	init_waitqueue_head(&msm->sync.af_status_wait);

	mutex_init(&msm->msm_lock);
	mutex_init(&msm->pict_pp_lock);
	mutex_init(&msm->msm_sem);

	cdev_init(&msm->cdev, &msm_fops);
	msm->cdev.owner = THIS_MODULE;

	rc = cdev_add(&msm->cdev, devno, 1);
	if (rc < 0)
		goto setup_cleanup_all;

	CDBG("msm_camera setup finishes!\n");
	return 0;

setup_cleanup_all:
	cdev_del(&msm->cdev);
setup_fail_return:
	return rc;
}
#else
static int msm_setup_cdev(struct msm_device *msm,
			int node,
			dev_t devno,
			const char *suffix,
			const struct file_operations *fops)
{
	int rc = -ENODEV;

	struct device *device =
		device_create(msm_class, NULL,
			devno, NULL,
			"%s%d", suffix, node);

	if (IS_ERR(device)) {
		rc = PTR_ERR(device);
		pr_err("msm_camera: error creating device: %d\n", rc);
		return rc;
	}

	cdev_init(&msm->cdev, fops);
	msm->cdev.owner = THIS_MODULE;

	rc = cdev_add(&msm->cdev, devno, 1);
	if (rc < 0) {
		pr_err("msm_camera: error adding cdev: %d\n", rc);
		device_destroy(msm_class, devno);
		return rc;
	}

	return rc;
}

static int msm_tear_down_cdev(struct msm_device *msm, dev_t devno)
{
	cdev_del(&msm->cdev);
	device_destroy(msm_class, devno);
	return 0;
}

int msm_v4l2_register(struct msm_v4l2_driver *drv)
{
	/* FIXME: support multiple sensors */
	if (list_empty(&msm_sensors))
		return -ENODEV;

	drv->sync = list_first_entry(&msm_sensors, struct msm_sync, list);
	drv->open      = __msm_open;
	drv->release   = __msm_release;
	drv->ctrl      = __msm_v4l2_control;
	drv->reg_pmem  = __msm_register_pmem;
	drv->get_frame = __msm_get_frame;
	drv->put_frame = __msm_put_frame_buf;
	drv->get_pict  = __msm_get_pic;
	drv->drv_poll  = __msm_poll_frame;

	return 0;
}
EXPORT_SYMBOL(msm_v4l2_register);

int msm_v4l2_unregister(struct msm_v4l2_driver *drv)
{
  drv->sync = NULL;
  return 0;
}
EXPORT_SYMBOL(msm_v4l2_unregister);

static int msm_sync_init(struct msm_sync *sync,
		struct platform_device *pdev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
				struct msm_sensor_ctrl *))
{
	int rc = 0;
	struct msm_sensor_ctrl sctrl;
	sync->sdata = pdev->dev.platform_data;

	spin_lock_init(&sync->msg_event_q_lock);
	INIT_LIST_HEAD(&sync->msg_event_q);
	init_waitqueue_head(&sync->msg_event_wait);

	spin_lock_init(&sync->prev_frame_q_lock);
	INIT_LIST_HEAD(&sync->prev_frame_q);
	init_waitqueue_head(&sync->prev_frame_wait);

	spin_lock_init(&sync->pict_frame_q_lock);
	INIT_LIST_HEAD(&sync->pict_frame_q);
	init_waitqueue_head(&sync->pict_frame_wait);

	wake_lock_init(&sync->wake_lock, WAKE_LOCK_IDLE, "msm_camera");

	rc = msm_camio_probe_on(pdev);
	if (rc < 0)
		return rc;
	rc = sensor_probe(sync->sdata, &sctrl);
	if (rc >= 0) {
		sync->pdev = pdev;
		sync->sctrl = sctrl;
	}
	msm_camio_probe_off(pdev);
	if (rc < 0) {
		pr_err("msm_camera: failed to initialize %s\n",
			sync->sdata->sensor_name);
		wake_lock_destroy(&sync->wake_lock);
		return rc;
	}

	sync->opencnt = 0;
	mutex_init(&sync->lock);
	CDBG("initialized %s\n", sync->sdata->sensor_name);
	return rc;
}

static int msm_sync_destroy(struct msm_sync *sync)
{
	wake_lock_destroy(&sync->wake_lock);
	return 0;
}

#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static long msm_control_proc(struct msm_ctrl_cmd *ctrlcmd,
	struct msm_device_t *vmsm)
{
	unsigned long flags;
	int timeout;
	long rc = 0;

	struct msm_queue_cmd_t *qcmd = NULL;
	struct msm_queue_cmd_t *rcmd = NULL;

	/* wake up config thread, 4 is for V4L2 application */
	qcmd = kmalloc(sizeof(struct msm_queue_cmd_t), GFP_ATOMIC);
	if (!qcmd) {
		CDBG("msm_control_proc: cannot allocate buffer\n");
		rc = -ENOMEM;
		goto end;
	}

	spin_lock_irqsave(&vmsm->sync.msg_event_queue_lock, flags);
	qcmd->type = MSM_CAM_Q_V4L2_REQ;
	qcmd->command = ctrlcmd;
	list_add_tail(&qcmd->list, &vmsm->sync.msg_event_queue);
	wake_up(&vmsm->sync.msg_event_wait);
	spin_unlock_irqrestore(&vmsm->sync.msg_event_queue_lock, flags);

	/* wait for config status */
	timeout = ctrlcmd->timeout_ms;
	CDBG("%s timeout = %d\n", __func__, timeout);
	if (timeout > 0) {
		rc =
			wait_event_timeout(
				vmsm->sync.ctrl_status_wait,
				msm_ctrl_stats_pending(vmsm),
				msecs_to_jiffies(timeout));

		CDBG("msm_control_proc: rc = %ld\n", rc);

		if (rc == 0) {
			CDBG("msm_control_proc: timed out\n");
			rc = -ETIMEDOUT;
			goto fail;
		}
	} else
		rc = wait_event_interruptible(
			vmsm->sync.ctrl_status_wait,
			msm_ctrl_stats_pending(vmsm));

	if (rc < 0)
		return -ERESTARTSYS;

	/* control command status is ready */
	spin_lock_irqsave(&vmsm->sync.ctrl_status_lock, flags);
	if (!list_empty(&vmsm->sync.ctrl_status_queue)) {
		rcmd = list_first_entry(
			&vmsm->sync.ctrl_status_queue,
			struct msm_queue_cmd_t,
			list);

		if (!rcmd) {
			spin_unlock_irqrestore(&vmsm->sync.ctrl_status_lock,
				flags);
			rc = -EAGAIN;
			goto end;
		}

		list_del(&(rcmd->list));
	}
	spin_unlock_irqrestore(&vmsm->sync.ctrl_status_lock, flags);

	memcpy(ctrlcmd->value,
		((struct msm_ctrl_cmd *)(rcmd->command))->value,
		((struct msm_ctrl_cmd *)(rcmd->command))->length);

	if (((struct msm_ctrl_cmd *)(rcmd->command))->length > 0)
		kfree(((struct msm_ctrl_cmd *)
					 (rcmd->command))->value);
	goto end;

fail:
	kfree(qcmd);
end:
	kfree(rcmd);
	CDBG("msm_control_proc: end rc = %ld\n", rc);
	return rc;
}

unsigned int msm_apps_poll(struct file *filep,
	struct poll_table_struct *pll_table, struct msm_device_t *pmsm)
{
	poll_wait(filep, &pmsm->sync.prev_frame_wait, pll_table);

	if (msm_frame_pending(pmsm))
		/* frame ready */
		return POLLIN | POLLRDNORM;

	return 0;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
long msm_register(struct msm_driver *drv, const char *id)
{
	long rc = -ENODEV;

	if (drv->vmsm)
		return -EINVAL;

	if (msm_camera) {
		if (*msm_camera)
			/* @todo to support multiple sensors */
			drv->vmsm = *msm_camera;
		else
			return rc;
	}

	mutex_lock(&drv->vmsm->msm_sem);

	if (drv->vmsm->apps_id == NULL) {
		drv->vmsm->apps_id = id;

		drv->init = msm_open_proc;
		drv->ctrl = msm_control_proc;
		drv->reg_pmem = msm_register_pmem_proc;
		drv->get_frame = msm_get_frame_proc;
		drv->put_frame = msm_put_frame_buf_proc;
		drv->get_pict  = msm_get_pict_proc;
		drv->drv_poll  = msm_apps_poll;
		rc = 0;
	} else
		rc = -EFAULT;

	mutex_unlock(&drv->vmsm->msm_sem);
	return rc;
}

long msm_unregister(struct msm_driver *drv,
	const char *id)
{
	long rc = -EFAULT;

	mutex_lock(&drv->vmsm->msm_sem);
	if (!strcmp(drv->vmsm->apps_id, id)) {
		drv->vmsm->apps_id = NULL;
		rc = 0;
	}
	mutex_unlock(&drv->vmsm->msm_sem);

	if (!rc)
		drv->vmsm = NULL;

	return rc;
}
#endif
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
static int msm_device_init(struct msm_device *pmsm,
		struct msm_sync *sync,
		int node)
{
	int dev_num = 3 * node;
	int rc = msm_setup_cdev(pmsm, node,
		MKDEV(MAJOR(msm_devno), dev_num),
		"control", &msm_fops_control);
	if (rc < 0) {
		pr_err("error creating control node: %d\n", rc);
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 1, node,
		MKDEV(MAJOR(msm_devno), dev_num + 1),
		"config", &msm_fops_config);
	if (rc < 0) {
		pr_err("error creating config node: %d\n", rc);
		msm_tear_down_cdev(pmsm, MKDEV(MAJOR(msm_devno),
				dev_num));
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 2, node,
		MKDEV(MAJOR(msm_devno), dev_num + 2),
		"frame", &msm_fops_frame);
	if (rc < 0) {
		pr_err("error creating frame node: %d\n", rc);
		msm_tear_down_cdev(pmsm,
			MKDEV(MAJOR(msm_devno), dev_num));
		msm_tear_down_cdev(pmsm + 1,
			MKDEV(MAJOR(msm_devno), dev_num + 1));
		return rc;
	}

	atomic_set(&pmsm[0].opened, 0);
	atomic_set(&pmsm[1].opened, 0);
	atomic_set(&pmsm[2].opened, 0);

	pmsm[0].sync = sync;
	pmsm[1].sync = sync;
	pmsm[2].sync = sync;

	return rc;
}
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
int msm_camera_drv_start(struct platform_device *dev)
{
	struct msm_camera_device_platform_data *pdata;
	struct msm_camera_sensor_info *sinfo;
	struct msm_sensor_ctrl_t sctrl;
	struct msm_device_t *pmsm = NULL;
	int i, cnt = 0;
	int rc = -ENODEV;

	if (!dev)
		return rc;

	pdata = dev->dev.platform_data;
	sinfo = pdata->sinfo;

	msm_camera =
		kzalloc(sizeof(struct msm_device_t *) * pdata->snum,
			GFP_KERNEL);
	if (!msm_camera) {
		rc = -ENOMEM;
		goto start_done;
	}

	rc = alloc_chrdev_region(&msm_devno, 0,
		pdata->snum, "msm_camera");
	if (rc < 0)
		goto start_region_fail;

	msm_class = class_create(THIS_MODULE, "msm_camera");
	if (IS_ERR(msm_class))
		goto start_class_fail;

	rc = msm_camio_probe_on(dev);
	if (rc < 0)
		goto start_io_fail;

	for (i = 0; i < pdata->snum; i++) {

		if (sinfo->sensor_probe(sinfo, &sctrl) >= 0) {

			pmsm = *(msm_camera + cnt) =
				kzalloc(sizeof(struct msm_device_t),
					GFP_KERNEL);
			if (!pmsm)
				continue;

			pmsm->pdev = dev;
			pmsm->sctrl = sctrl;
			pmsm->sidx  = i;

			rc = msm_setup_cdevs(pmsm,
				MKDEV(MAJOR(msm_devno), cnt));

			if (rc >= 0) {
				cnt++;
				if (cnt > 0)
					break;
			} else {
				kfree(pmsm);
				*(msm_camera + cnt) = NULL;
				continue;
			}
		}

		sinfo++;
	}
	msm_camio_probe_off(dev);

	if (cnt > 0) {
		msm_camvfe_init();
		goto start_done;
	}

start_io_fail:
	class_destroy(msm_class);
start_class_fail:
	unregister_chrdev_region(msm_devno, 1);
start_region_fail:
	kfree(msm_camera);
	CDBG("FAIL: %s %s:%d\n", __FILE__, __func__, __LINE__);
start_done:
	CDBG("DONE: %s rc = %d\n", __func__, rc);
	return rc;
}
EXPORT_SYMBOL(msm_camera_drv_start);
#else
int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
			struct msm_sensor_ctrl *))
{
	struct msm_device *pmsm = NULL;
	struct msm_sync *sync;
	int rc = -ENODEV;
	static int camera_node;

	if (camera_node >= MSM_MAX_CAMERA_SENSORS) {
		pr_err("msm_camera: too many camera sensors\n");
		return rc;
	}

	if (!msm_class) {
		/* There are three device nodes per sensor */

		msm_class = class_create(THIS_MODULE, "msm_camera");
		if (IS_ERR(msm_class)) {
			rc = PTR_ERR(msm_class);
			pr_err("msm_camera: create device class failed: %d\n",
				rc);
			return rc;
		}

	  rc = alloc_chrdev_region(&msm_devno, 0,
				3 * MSM_MAX_CAMERA_SENSORS,
				"msm_camera");
		if (rc < 0) {
			pr_err("msm_camera: failed to allocate chrdev: %d\n",
				rc);
			return rc;
		}
	}

	pmsm = kzalloc(sizeof(struct msm_device) * 3 +
			sizeof(struct msm_sync), GFP_ATOMIC);
	if (!pmsm)
		return -ENOMEM;
	sync = (struct msm_sync *)(pmsm + 3);

	rc = msm_sync_init(sync, dev, sensor_probe);
	if (rc < 0) {
		kfree(pmsm);
		return rc;
	}

	CDBG("setting camera node %d\n", camera_node);
	rc = msm_device_init(pmsm, sync, camera_node);
	if (rc < 0) {
		msm_sync_destroy(sync);
		kfree(pmsm);
		return rc;
	}

	camera_node++;
	list_add(&sync->list, &msm_sensors);
	return rc;
}
EXPORT_SYMBOL(msm_camera_drv_start);
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
int msm_camera_drv_remove(struct platform_device *dev)
{
	struct msm_camera_device_platform_data *pdata;
	struct msm_camera_sensor_info *sinfo;
	struct msm_device_t *pmsm = NULL;
	int i;

	pdata = dev->dev.platform_data;
	sinfo = pdata->sinfo;

	for (i = 0; i < pdata->snum; i++) {
		pmsm = *(msm_camera + i);
		if (pmsm) {
			cdev_del(&pmsm->cdev);
			kfree(pmsm);
			/* Not necessary; do it anyway */
			*(msm_camera + i) = NULL;
		}
	}

	pdata = dev->dev.platform_data;
	sinfo = pdata->sinfo;

	class_destroy(msm_class);
	unregister_chrdev_region(msm_devno, 1);

	kfree(msm_camera);

	return 0;
}
#endif
