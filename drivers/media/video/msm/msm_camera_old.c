/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/* FIXME: management of mutexes */
/* FIXME: msm_pmem_region_lookup return values */
/* FIXME: way too many copy to/from user */
/* FIXME: does region->active mean free */
/* FIXME: check limits on command lenghts passed from userspace */
/* FIXME: __msm_release: which queues should we flush when opencnt != 0 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <mach/board.h>

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <linux/sched.h>

#define MSM_MAX_CAMERA_SENSORS 5
DEFINE_MUTEX(ctrl_cmd_lock);

#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

static struct class *msm_class;
static dev_t msm_devno;
static LIST_HEAD(msm_sensors);

#define __CONTAINS(r, v, l, field) ({				\
	typeof(r) __r = r;					\
	typeof(v) __v = v;					\
	typeof(v) __e = __v + l;				\
	int res = __v >= __r->field &&				\
		__e <= __r->field + __r->len;			\
	res;							\
})

#define CONTAINS(r1, r2, field) ({				\
	typeof(r2) __r2 = r2;					\
	__CONTAINS(r1, __r2->field, __r2->len, field);		\
})

#define IN_RANGE(r, v, field) ({				\
	typeof(r) __r = r;					\
	typeof(v) __vv = v;					\
	int res = ((__vv >= __r->field) &&			\
		(__vv < (__r->field + __r->len)));		\
	res;							\
})

#define OVERLAPS(r1, r2, field) ({				\
	typeof(r1) __r1 = r1;					\
	typeof(r2) __r2 = r2;					\
	typeof(__r2->field) __v = __r2->field;			\
	typeof(__v) __e = __v + __r2->len - 1;			\
	int res = (IN_RANGE(__r1, __v, field) ||		\
		   IN_RANGE(__r1, __e, field));                 \
	res;							\
})

#define MSM_DRAIN_QUEUE_NOSYNC(sync, name) do {			\
	struct msm_queue_cmd *qcmd = NULL;			\
	CDBG("%s: draining queue "#name"\n", __func__);		\
	while (!list_empty(&(sync)->name)) {			\
		qcmd = list_first_entry(&(sync)->name,		\
			struct msm_queue_cmd, list);		\
		list_del_init(&qcmd->list);			\
		if (qcmd->on_heap)				\
			kfree(qcmd);				\
	};							\
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

static int check_pmem_info(struct msm_pmem_info *info, int len)
{
	if (info->offset < len &&
	    info->offset + info->len <= len &&
	    info->y_off < len &&
	    info->cbcr_off < len)
		return 0;

	pr_err("%s: check failed: off %d len %d y %d cbcr %d (total len %d)\n",
		__func__,
		info->offset,
		info->len,
		info->y_off,
		info->cbcr_off,
		len);
	return -EINVAL;
}

static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info)
{
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

	if (!info->len)
		info->len = len;

	rc = check_pmem_info(info, len);
	if (rc < 0)
		return rc;

	paddr += info->offset;
	len = info->len;

	if (check_overlap(ptype, paddr, len) < 0)
		return -EINVAL;

	CDBG("%s: type %d, paddr 0x%lx, vaddr 0x%lx\n",
		__func__,
		info->type, paddr, (unsigned long)info->vaddr);

	region = kmalloc(sizeof(struct msm_pmem_region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	INIT_HLIST_NODE(&region->list);

	region->paddr = paddr;
	region->len = len;
	region->file = file;
	memcpy(&region->info, info, sizeof(region->info));

	hlist_add_head(&(region->list), ptype);

	return 0;
}

/* return of 0 means failure */
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	int pmem_type, struct msm_pmem_region *reg, uint8_t maxcount)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node, *n;

	uint8_t rc = 0;

	regptr = reg;

	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		if (region->info.type == pmem_type && region->info.vfe_can_write) {
			*regptr = *region;
			rc += 1;
			if (rc >= maxcount)
				break;
			regptr++;
		}
	}

	return rc;
}

static int msm_pmem_frame_ptov_lookup(struct msm_sync *sync,
		unsigned long pyaddr,
		unsigned long pcbcraddr,
		struct msm_pmem_info *pmem_info)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->frame, list) {
		if (pyaddr == (region->paddr + region->info.y_off) &&
				pcbcraddr == (region->paddr +
						region->info.cbcr_off) &&
				region->info.vfe_can_write) {
			/* offset since we could pass vaddr inside
			 * a registerd pmem buffer
			 */
			memcpy(pmem_info, &region->info, sizeof(*pmem_info));
			region->info.vfe_can_write = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static unsigned long msm_pmem_stats_ptov_lookup(struct msm_sync *sync,
		unsigned long addr, int *fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->stats, list) {
		if (addr == region->paddr && region->info.vfe_can_write) {
			/* offset since we could pass vaddr inside a
			 * registered pmem buffer */
			*fd = region->info.fd;
			region->info.vfe_can_write = 0;
			return (unsigned long)(region->info.vaddr);
		}
	}

	return 0;
}

static unsigned long msm_pmem_frame_vtop_lookup(struct msm_sync *sync,
		unsigned long buffer,
		uint32_t yoff, uint32_t cbcroff, int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region,
		node, n, &sync->frame, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
				(region->info.y_off == yoff) &&
				(region->info.cbcr_off == cbcroff) &&
				(region->info.fd == fd) &&
				(region->info.vfe_can_write == 0)) {
			region->info.vfe_can_write = 1;
			return region->paddr;
		}
	}

	return 0;
}

static unsigned long msm_pmem_stats_vtop_lookup(
		struct msm_sync *sync,
		unsigned long buffer,
		int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->stats, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
				(region->info.fd == fd) &&
				region->info.vfe_can_write == 0) {
			region->info.vfe_can_write = 1;
			return region->paddr;
		}
	}

	return 0;
}

static int __msm_pmem_table_del(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
{
	int rc = 0;
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
		hlist_for_each_entry_safe(region, node, n,
			&sync->frame, list) {

			if (pinfo->type == region->info.type &&
					pinfo->vaddr == region->info.vaddr &&
					pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		hlist_for_each_entry_safe(region, node, n,
			&sync->stats, list) {

			if (pinfo->type == region->info.type &&
					pinfo->vaddr == region->info.vaddr &&
					pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_pmem_table_del(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_pmem_table_del(sync, &info);
}

static int __msm_get_frame(struct msm_sync *sync,
		struct msm_frame *frame)
{
	unsigned long flags;
	int rc = 0;

	struct msm_pmem_info pmem_info;
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_vfe_phy_info *pphy;

	spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
	if (!list_empty(&sync->prev_frame_q)) {
		qcmd = list_first_entry(&sync->prev_frame_q,
			struct msm_queue_cmd, list);
		list_del_init(&qcmd->list);
	}
	spin_unlock_irqrestore(&sync->prev_frame_q_lock, flags);

	if (!qcmd) {
		pr_err("%s: no preview frame.\n", __func__);
		return -EAGAIN;
	}

	pphy = (struct msm_vfe_phy_info *)(qcmd->command);

	rc = msm_pmem_frame_ptov_lookup(sync,
			pphy->y_phy,
			pphy->cbcr_phy,
			&pmem_info);

	if (rc < 0) {
		pr_err("%s: cannot get frame, invalid lookup address "
			"y %x cbcr %x\n",
			__func__,
			pphy->y_phy,
			pphy->cbcr_phy);
		goto err;
	}

	frame->buffer = (unsigned long)pmem_info.vaddr;
	frame->y_off = pmem_info.y_off;
	frame->cbcr_off = pmem_info.cbcr_off;
	frame->fd = pmem_info.fd;

	CDBG("%s: y %x, cbcr %x, qcmd %x, virt_addr %x\n",
		__func__,
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);

err:
	if (qcmd && qcmd->on_heap)
		kfree(qcmd);
	return rc;
}

static int msm_get_frame(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_frame frame;

	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	rc = __msm_get_frame(sync, &frame);
	if (rc < 0)
		return rc;

	if (sync->croplen) {
		if (frame.croplen != sync->croplen) {
			pr_err("%s: invalid frame croplen %d,"
				"expecting %d\n",
				__func__,
				frame.croplen,
				sync->croplen);
			return -EINVAL;
		}

		if (copy_to_user((void *)frame.cropinfo,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			return -EFAULT;
		}
	}

	if (copy_to_user((void *)arg,
				&frame, sizeof(struct msm_frame))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	CDBG("%s: got frame\n", __func__);

	return rc;
}

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

	CDBG("%s: rc %d\n", __func__, rc);
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

	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

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
			pr_err("%s: wait_event error %d\n", __func__, rc);
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
		pr_err("%s: cannot allocate buffer udata len %d\n",
			__func__, udata.length);
		rc = -ENOMEM;
		goto end;
	}

	qcmd->on_heap = 1;
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
	if (qcmd && qcmd->on_heap)
		kfree(qcmd);
	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

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

	CDBG("%s: timeout %d\n", __func__, timeout);
	rc = wait_event_interruptible_timeout(
			sync->msg_event_wait,
			!list_empty_careful(&sync->msg_event_q),
			msecs_to_jiffies(timeout));
	if (list_empty_careful(&sync->msg_event_q)) {
		if (rc == 0)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("%s: error %d\n", __func__, rc);
			return rc;
		}
	}
	CDBG("%s: returned from wait: %d\n", __func__, rc);

	spin_lock_irqsave(&sync->msg_event_q_lock, flags);
	BUG_ON(list_empty(&sync->msg_event_q));
	qcmd = list_first_entry(&sync->msg_event_q,
			struct msm_queue_cmd, list);
	list_del_init(&qcmd->list);
	spin_unlock_irqrestore(&sync->msg_event_q_lock, flags);

	CDBG("%s: received from DSP %d\n", __func__, qcmd->type);

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

		CDBG("%s: qcmd->type %d length %d msd_id %d\n", __func__,
			qcmd->type,
			se.stats_event.len,
			se.stats_event.msg_id);

		if ((data->type == VFE_MSG_STATS_AF) ||
				(data->type == VFE_MSG_STATS_WE)) {

			stats.buffer =
			msm_pmem_stats_ptov_lookup(sync,
					data->phy.sbuf_phy,
					&(stats.fd));
			if (!stats.buffer) {
				pr_err("%s: msm_pmem_stats_ptov_lookup error\n",
					__func__);
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
				goto failure;
			}
		} else if (data->type == VFE_MSG_OUTPUT1 ||
			data->type == VFE_MSG_OUTPUT2) {
			if (sync->pict_pp & PP_PREV) {
				struct msm_pmem_info pinfo;
				struct msm_postproc buf;

				sync->pp_sync_flag = 0;
				CDBG("%s: preview PP sync->pict_pp %d\n",
					__func__, sync->pict_pp);

				rc = msm_pmem_frame_ptov_lookup(sync,
						data->phy.y_phy,
						data->phy.cbcr_phy,
						&pinfo);
				if (rc < 0) {
					CDBG("%s: msm_pmem_frame_ptov_lookup "
						"failed\n", __func__);
					rc = -EIO;
					goto failure;
				}

				buf.fmain.buffer = (unsigned long)pinfo.vaddr;
				buf.fmain.y_off = pinfo.y_off;
				buf.fmain.cbcr_off = pinfo.cbcr_off;
				buf.fmain.fd = pinfo.fd;

				CDBG("%s: buf %ld fd %d\n",
					__func__, buf.fmain.buffer,
					buf.fmain.fd);
				if (copy_to_user((void *)(se.stats_event.data),
						&(buf.fmain),
						sizeof(struct msm_frame))) {
					ERR_COPY_TO_USER();
					rc = -EFAULT;
					goto failure;
				}
			} else {
				CDBG("%s: msm->pp_sync_flag %d \n",
					 __func__, sync->pp_sync_flag);

				if (sync->pp_sync_flag == 1) {
					qcmd_frame = kmalloc(sizeof(
						struct msm_queue_cmd) +
						sizeof(struct msm_vfe_phy_info),
						GFP_KERNEL);
					if (!qcmd_frame) {
						rc = -ENOMEM;
						goto failure;
					}
					fphy = (struct msm_vfe_phy_info *)
							(qcmd_frame + 1);

					*fphy = data->phy;
					qcmd_frame->type = MSM_CAM_Q_VFE_MSG;
					qcmd_frame->command = fphy;
					qcmd_frame->on_heap = 1;

					CDBG("%s: phy_y %x phy_cbcr %x\n",
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
					CDBG("%s: woke up frame thread\n",
						__func__);
				} else {
					CDBG("%s: for config thread.\n",
						__func__);
					if (copy_to_user(
						(void *)(se.stats_event.data),
							data->extdata,
							data->extlen)) {
						ERR_COPY_TO_USER();
						rc = -EFAULT;
						goto failure;
					}
				}

			}
		} else if (data->type == VFE_MSG_SNAPSHOT) {
			if (sync->pict_pp & PP_SNAP) {
				struct msm_postproc buf;
				struct msm_pmem_region region;
				memset(&region, 0, sizeof(region));
				buf.fmnum = msm_pmem_region_lookup(&sync->frame,
						MSM_PMEM_MAINIMG,
						&region, 1);
				if (buf.fmnum == 1) {
					buf.fmain.buffer =
						(uint32_t)region.info.vaddr;
					buf.fmain.y_off  = region.info.y_off;
					buf.fmain.cbcr_off =
						region.info.cbcr_off;
					buf.fmain.fd = region.info.fd;
				} else {
					buf.fmnum = msm_pmem_region_lookup(
							&sync->frame,
							MSM_PMEM_RAW_MAINIMG,
							&region, 1);
					if (buf.fmnum == 1) {
						buf.fmain.path =
							MSM_FRAME_PREV_2;
						buf.fmain.buffer =
						(uint32_t)region.info.vaddr;

						buf.fmain.fd = region.info.fd;
					} else {
						pr_err("%s: pmem lookup fail\n",
							__func__);
						rc = -EINVAL;
						goto failure;
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
			CDBG("%s: snapshot copy_to_user!\n", __func__);
		}
		break;

	case MSM_CAM_Q_CTRL:
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("%s: qcmd->type %d length %d\n", __func__,
			qcmd->type, ctrl->length);

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

		CDBG("%s: qcmd->type %d\n", __func__, qcmd->type, ctrl->length);

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
		goto failure;
	}

failure:
	if (qcmd && qcmd->on_heap)
		kfree(qcmd);

	CDBG("%s: %d\n", __func__, rc);
	return rc;
}

static int msm_ctrl_cmd_done(struct msm_control_device *ctrl_pmsm,
		void __user *arg)
{
	unsigned long flags;
	int rc = 0;

	struct msm_ctrl_cmd udata, *ctrlcmd;
	struct msm_queue_cmd *qcmd = NULL;

	if (copy_from_user(&udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto end;
	}

	qcmd = kmalloc(sizeof(struct msm_queue_cmd) +
			sizeof(struct msm_ctrl_cmd) + udata.length,
			GFP_KERNEL);
	if (!qcmd) {
		pr_err("%s: failed to allocate %d bytes\n", __func__,
			udata.length);
		rc = -ENOMEM;
		goto end;
	}

	qcmd->command = ctrlcmd = (struct msm_ctrl_cmd *)(qcmd + 1);
	qcmd->on_heap = 1;
	*ctrlcmd = udata;
	if (udata.length > 0) {
		ctrlcmd->value = ctrlcmd + 1;
		if (copy_from_user(ctrlcmd->value,
					(void *)udata.value,
					udata.length)) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
			if (qcmd && qcmd->on_heap)
				kfree(qcmd);
			goto end;
		}
	} else
		ctrlcmd->value = NULL;

end:
	CDBG("%s: end rc %d\n", __func__, rc);
	if (!rc) {
		/* wake up control thread */
		spin_lock_irqsave(&ctrl_pmsm->ctrl_q.ctrl_status_q_lock, flags);
		list_add_tail(&qcmd->list, &ctrl_pmsm->ctrl_q.ctrl_status_q);
		wake_up(&ctrl_pmsm->ctrl_q.ctrl_status_wait);
		spin_unlock_irqrestore(&ctrl_pmsm->ctrl_q.ctrl_status_q_lock,
					flags);
	}

	return rc;
}

static int msm_config_vfe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;
	struct msm_pmem_region region[8];
	struct axidata axi_data;

	if (!sync->vfefn.vfe_config) {
		pr_err("%s: no vfe_config!\n", __func__);
		return -EIO;
	}

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	memset(&axi_data, 0, sizeof(axi_data));

	CDBG("%s: cmd_type %d\n", __func__, cfgcmd.cmd_type);

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
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AF_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats,
					MSM_PMEM_AF, &region[0],
					NUM_AF_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats,
					MSM_PMEM_AEC_AWB, &region[0],
					NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_GENERAL:
	case CMD_STATS_DISABLE:
		return sync->vfefn.vfe_config(&cfgcmd, NULL);
	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd.cmd_type);
	}

	return -EINVAL;
}

static int msm_frame_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {
	case CMD_AXI_CFG_OUT1:
		pmem_type = MSM_PMEM_OUTPUT1;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_AXI_CFG_OUT2:
		pmem_type = MSM_PMEM_OUTPUT2;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error (empty %d)\n",
				__func__, __LINE__,
				hlist_empty(&sync->frame));
			return -EINVAL;
		}
		break;

	case CMD_AXI_CFG_SNAP_O1_AND_O2:
		pmem_type = MSM_PMEM_THUMBNAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[axi_data.bufnum1], 8);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_RAW_PICT_AXI_CFG:
		pmem_type = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->frame, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum2) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_GENERAL:
		data = NULL;
		break;

	default:
		pr_err("%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	axi_data.region = &region[0];

	/* send the AXI configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, data);

	return rc;
}

static int msm_get_sensor_info(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_sensor_info *sdata;

	if (copy_from_user(&info,
			arg,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	sdata = sync->pdev->dev.platform_data;
	CDBG("%s: sensor_name %s\n", __func__, sdata->sensor_name);

	memcpy(&info.name[0],
		sdata->sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled = 0;

	/* copy back to user space */
	if (copy_to_user((void *)arg,
			&info,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	return rc;
}

static int __msm_put_frame_buf(struct msm_sync *sync,
		struct msm_frame *pb)
{
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	int rc = -EIO;

	pphy = msm_pmem_frame_vtop_lookup(sync,
		pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd);

	if (pphy != 0) {
		CDBG("%s: rel: vaddr %lx, paddr %lx\n",
			__func__,
			pb->buffer, pphy);
		cfgcmd.cmd_type = CMD_FRAME_BUF_RELEASE;
		cfgcmd.value    = (void *)pb;
		if (sync->vfefn.vfe_config)
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
	} else {
		pr_err("%s: msm_pmem_frame_vtop_lookup failed\n",
			__func__);
		rc = -EINVAL;
	}

	return rc;
}

static int msm_put_frame_buffer(struct msm_sync *sync, void __user *arg)
{
	struct msm_frame buf_t;

	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_put_frame_buf(sync, &buf_t);
}

static int __msm_register_pmem(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
{
	int rc = 0;

	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
		rc = msm_pmem_table_add(&sync->frame, pinfo);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		rc = msm_pmem_table_add(&sync->stats, pinfo);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_register_pmem(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_register_pmem(sync, &info);
}

static int msm_stats_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;

	struct msm_pmem_region region[3];
	int pmem_type = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

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
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	if (cfgcmd->cmd_type != CMD_GENERAL) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->stats, pmem_type,
				&region[0], NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
	}

	/* send the AEC/AWB STATS configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, &axi_data);

	return rc;
}

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

	CDBG("%s\n", __func__);
	pphy = msm_pmem_stats_vtop_lookup(sync, buf.buffer, buf.fd);

	if (pphy != 0) {
		if (buf.type == STAT_AEAW)
			cfgcmd.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else {
			pr_err("%s: invalid buf type %d\n",
				__func__,
				buf.type);
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd.value = (void *)&buf;

		if (sync->vfefn.vfe_config) {
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
			if (rc < 0)
				pr_err("%s: vfe_config error %d\n",
					__func__, rc);
		} else
			pr_err("%s: vfe_config is NULL\n", __func__);
	} else {
		pr_err("%s: NULL physical address\n", __func__);
		rc = -EINVAL;
	}

put_done:
	return rc;
}

static int msm_axi_config(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	switch (cfgcmd.cmd_type) {
	case CMD_AXI_CFG_OUT1:
	case CMD_AXI_CFG_OUT2:
	case CMD_AXI_CFG_SNAP_O1_AND_O2:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(sync, &cfgcmd);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(sync, &cfgcmd);

	default:
		pr_err("%s: unknown command type %d\n",
			__func__,
			cfgcmd.cmd_type);
		return -EINVAL;
	}

	return 0;
}

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
			pr_err("%s: rc %d\n", __func__, rc);
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

	if (qcmd && qcmd->on_heap)
		kfree(qcmd);
	return rc;
}

static int msm_get_pic(struct msm_sync *sync, void __user *arg)
{
	struct msm_ctrl_cmd ctrlcmd_t;
	int rc;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	rc = __msm_get_pic(sync, &ctrlcmd_t);
	if (rc < 0)
		return rc;

	if (sync->croplen) {
		if (ctrlcmd_t.length != sync->croplen) {
			pr_err("%s: invalid len %d < %d\n",
				__func__,
				ctrlcmd_t.length,
				sync->croplen);
			return -EINVAL;
		}
		if (copy_to_user(ctrlcmd_t.value,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			return -EFAULT;
		}
	}
	CDBG("%s: copy snapshot frame to user\n", __func__);
	if (copy_to_user((void *)arg,
		&ctrlcmd_t,
		sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}
	return 0;
}

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

static int msm_pict_pp_start(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	uint8_t enable;
	CDBG("%s: sync->pict_pp %d enable %d\n", __func__,
		sync->pict_pp, enable);
	if (copy_from_user(&enable, arg, sizeof(enable))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
	} else {
		enable &= PP_MASK;
		if (sync->pict_pp & enable) {
			pr_err("%s: postproc %x is already enabled\n",
				__func__, sync->pict_pp & enable);
			rc = -EINVAL;
		} else
			sync->pict_pp |= enable;
	}
	return rc;
}

static int msm_pict_pp_done(struct msm_sync *sync, void __user *arg)
{
	struct msm_ctrl_cmd udata;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_queue_cmd *qcmd;
	struct msm_vfe_phy_info *fphy;
	struct msm_frame frame;
	unsigned long pphy;

	unsigned long flags;
	int rc = 0;

	if (copy_from_user(&udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto done;
	}

	udata.status &= PP_MASK;
	/* udata.status has a bit set for each PP that needs to be turned
	 * off.  If nothing that's running is being turned off, return
	 * success immediately.
	 */
	if (!(sync->pict_pp & udata.status)) {
		pr_warning("%s: pp not in progress for %x\n", __func__,
			udata.status);
		rc = -EINVAL;
		goto done;
	}

	qcmd = kzalloc(sizeof(struct msm_queue_cmd) +
			sizeof(struct msm_ctrl_cmd) +
			sizeof(struct msm_vfe_phy_info) +
			((udata.status & PP_PREV) ? udata.length : 0),
			GFP_KERNEL);
	if (!qcmd) {
		pr_err("%s: out of memory\n", __func__);
		rc = -ENOMEM;
		goto done;
	}

	qcmd->on_heap = 1;
	ctrlcmd = (struct msm_ctrl_cmd *)(qcmd + 1);

	if (udata.status & PP_PREV) {
		if (sync->pict_pp & PP_PREV) {
			fphy = (struct msm_vfe_phy_info *)(ctrlcmd + 1);
			if (udata.length) {
				ctrlcmd->value = fphy + 1;
				if (copy_from_user(ctrlcmd->value,
							udata.value,
							udata.length)) {
					ERR_COPY_FROM_USER();
					if (qcmd && qcmd->on_heap)
						kfree(qcmd);
					rc = -EFAULT;
					goto done;
				}
			}
			memcpy(&frame, ctrlcmd->value, udata.length);
			CDBG("%s: buffer %ld, y_off %u, cbcr_off %u, fd %d\n",
				__func__,
				frame.buffer, frame.y_off,
				frame.cbcr_off, frame.fd);
			pphy = msm_pmem_frame_vtop_lookup(sync, frame.buffer,
				frame.y_off, frame.cbcr_off, frame.fd);
			CDBG("%s: vaddr %lx, pphy %lx\n", __func__,
				frame.buffer, pphy);
			if (pphy != 0) {
				fphy->y_phy = pphy + frame.y_off;
				fphy->cbcr_phy = pphy + frame.cbcr_off;
				fphy->sbuf_phy = 0;
			}
			CDBG("%s: vaddr %lx, y_phy %x cbcr_phy %x\n",
				__func__,
				frame.buffer, fphy->y_phy, fphy->cbcr_phy);
			qcmd->type    = MSM_CAM_Q_VFE_MSG;
			qcmd->command = fphy;
			CDBG("%s: phy_y %x, phy_cbcr %x\n", __func__,
				fphy->y_phy, fphy->cbcr_phy);
			spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
			list_add_tail(&qcmd->list, &sync->prev_frame_q);
			spin_unlock_irqrestore(&sync->prev_frame_q_lock,
				flags);
			CDBG("%s: waking up frame thread\n", __func__);
			wake_up(&sync->prev_frame_wait);
		}
	} else if (udata.status & PP_SNAP) {
		if (sync->pict_pp & PP_SNAP) {
			ctrlcmd->type = udata.type;
			ctrlcmd->status = udata.status;

			qcmd->type = MSM_CAM_Q_VFE_MSG;
			qcmd->command = ctrlcmd =
				(struct msm_ctrl_cmd *)(qcmd + 1);
			memset(ctrlcmd, 0, sizeof(struct msm_ctrl_cmd));
			ctrlcmd->type = udata.type;
			ctrlcmd->status = udata.status;

			spin_lock_irqsave(&sync->pict_frame_q_lock, flags);
			list_add_tail(&qcmd->list, &sync->pict_frame_q);
			spin_unlock_irqrestore(&sync->pict_frame_q_lock, flags);
			CDBG("%s: waking up picture thread\n", __func__);
			wake_up(&sync->pict_frame_wait);
		}
	}

	sync->pict_pp &= ~udata.status;

done:
	return rc;
}

static long msm_ioctl_common(struct msm_device *pmsm,
		unsigned int cmd,
		void __user *argp)
{
	CDBG("%s\n", __func__);
	switch (cmd) {
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(pmsm->sync, argp);
	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(pmsm->sync, argp);
	default:
		return -EINVAL;
	}
}

int msm_camera_flash(struct msm_sync *sync, int level)
{
	int flash_level;

	if (!sync->sdata->camera_flash) {
		pr_err("%s: camera flash is not supported.\n", __func__);
		return -EINVAL;
	}

	if (!sync->sdata->num_flash_levels) {
		pr_err("%s: no flash levels.\n", __func__);
		return -EINVAL;
	}

	switch (level) {
	case MSM_CAMERA_LED_HIGH:
		flash_level = sync->sdata->num_flash_levels - 1;
		break;
	case MSM_CAMERA_LED_LOW:
		flash_level = sync->sdata->num_flash_levels / 2;
		break;
	case MSM_CAMERA_LED_OFF:
		flash_level = 0;
		break;
	default:
		pr_err("%s: invalid flash level %d.\n", __func__, level);
		return -EINVAL;
	}

	return sync->sdata->camera_flash(level);
}

static long msm_ioctl_config(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_device *pmsm = filep->private_data;

	CDBG("%s: cmd %d\n", __func__, _IOC_NR(cmd));

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

	case MSM_CAM_IOCTL_PP: {
		rc = msm_pict_pp_start(pmsm->sync, argp);
		break;
	}

	case MSM_CAM_IOCTL_PP_DONE:
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
			rc = msm_camera_flash(pmsm->sync, led_state);
		break;
	}

	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	CDBG("%s: cmd %d DONE\n", __func__, _IOC_NR(cmd));
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
		mutex_lock(&ctrl_cmd_lock);
		rc = msm_control(ctrl_pmsm, 1, argp);
		mutex_unlock(&ctrl_cmd_lock);
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
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		break;
	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	return rc;
}

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

		kfree(sync->cropinfo);
		sync->cropinfo = NULL;
		sync->croplen = 0;

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

		wake_unlock(&sync->wake_lock);

		sync->apps_id = NULL;
		CDBG("%s: completed\n", __func__);
	}
	mutex_unlock(&sync->lock);

	return 0;
}

static int msm_release_config(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_device *pmsm = filep->private_data;
	CDBG("%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	atomic_set(&pmsm->opened, 0);
	return rc;
}

static int msm_release_control(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_device *pmsm = ctrl_pmsm->pmsm;
	CDBG("%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
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
	CDBG("%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
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
	CDBG("%s\n", __func__);
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

static void *msm_vfe_sync_alloc(int size,
			void *syncdata __attribute__((unused)),
			gfp_t gfp)
{
	struct msm_queue_cmd *qcmd =
		kmalloc(sizeof(struct msm_queue_cmd) + size, gfp);
	if (qcmd) {
		qcmd->on_heap = 1;
		return qcmd + 1;
	}
	return NULL;
}

static void msm_vfe_sync_free(void *ptr)
{
	if (ptr) {
		struct msm_queue_cmd *qcmd =
			(struct msm_queue_cmd *)ptr;
		qcmd--;
		if (qcmd->on_heap)
			kfree(qcmd);
	}
}

/*
 * This function executes in interrupt context.
 */

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
		enum msm_queue qtype, void *syncdata,
		gfp_t gfp)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_queue_cmd *qcmd_frame = NULL;
	struct msm_vfe_phy_info *fphy;

	unsigned long flags;
	struct msm_sync *sync = (struct msm_sync *)syncdata;
	if (!sync) {
		pr_err("%s: no context in dsp callback.\n", __func__);
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;

	if (qtype == MSM_CAM_Q_VFE_MSG) {
		CDBG("%s: vdata->type %d\n", __func__, vdata->type);
		switch (vdata->type) {
		case VFE_MSG_OUTPUT1:
		case VFE_MSG_OUTPUT2:
			if (sync->pict_pp & PP_PREV) {
				CDBG("%s: PP_PREV in progess: "
					"phy_y %x phy_cbcr %x\n",
					__func__,
					vdata->phy.y_phy,
					vdata->phy.cbcr_phy);
				sync->pp_sync_flag = 1;
				break;
			}
			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd) +
					sizeof(struct msm_vfe_phy_info),
					gfp);
			if (!qcmd_frame)
				goto mem_fail;
			fphy = (struct msm_vfe_phy_info *)(qcmd_frame + 1);
			*fphy = vdata->phy;

			qcmd_frame->type = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = fphy;
			qcmd_frame->on_heap = 1;

			CDBG("%s: qcmd_frame %x phy_y %x, phy_cbcr %x\n",
				__func__,
				qcmd_frame, fphy->y_phy, fphy->cbcr_phy);

			spin_lock_irqsave(&sync->prev_frame_q_lock, flags);
			list_add_tail(&qcmd_frame->list, &sync->prev_frame_q);
			wake_up(&sync->prev_frame_wait);
			spin_unlock_irqrestore(&sync->prev_frame_q_lock, flags);
			CDBG("%s: woke up frame thread\n", __func__);
			break;
		case VFE_MSG_SNAPSHOT:
			if ((sync->pict_pp & PP_SNAP) ||
					(sync->pict_pp & PP_RAW_SNAP)) {
				CDBG("%s: sync_data, sync->pict_pp %d\n",
					__func__, sync->pict_pp);
				break;
			}

			CDBG("%s: snapshot pp %d\n", __func__, sync->pict_pp);
			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd),
					gfp);
			if (!qcmd_frame)
				goto mem_fail;
			qcmd_frame->on_heap = 1;
			qcmd_frame->type = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = NULL;
				spin_lock_irqsave(&sync->pict_frame_q_lock,
				flags);
			list_add_tail(&qcmd_frame->list, &sync->pict_frame_q);
			wake_up(&sync->pict_frame_wait);
			spin_unlock_irqrestore(&sync->pict_frame_q_lock, flags);
			CDBG("%s: woke up picture thread\n", __func__);
			break;
		default:
			/* too chatty, plus not an error */
			CDBG("%s: qtype %d not handled\n",
				__func__, vdata->type);
			break;
		}
	}

	qcmd->command = (void *)vdata;

	spin_lock_irqsave(&sync->msg_event_q_lock, flags);
	list_add_tail(&qcmd->list, &sync->msg_event_q);
	wake_up(&sync->msg_event_wait);
	spin_unlock_irqrestore(&sync->msg_event_q_lock, flags);
	CDBG("%s: woke up config thread\n", __func__);
	return;

mem_fail:
	if (qcmd->on_heap)
		kfree(qcmd);
}

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
	.vfe_alloc = msm_vfe_sync_alloc,
	.vfe_free = msm_vfe_sync_free,
};

static int __msm_open(struct msm_sync *sync, const char *const apps_id)
{
	int rc = 0;

	mutex_lock(&sync->lock);
	if (sync->apps_id && strcmp(sync->apps_id, apps_id)) {
		pr_err("%s(%s): sensor %s is already opened for %s\n",
			__func__,
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
				pr_err("%s: vfe_init failed at %d\n",
					__func__, rc);
				goto msm_open_done;
			}
			rc = sync->sctrl.s_init(sync->sdata);
			if (rc < 0) {
				pr_err("%s: sensor init failed: %d\n",
					__func__, rc);
				goto msm_open_done;
			}
		} else {
			pr_err("%s: no sensor init func\n", __func__);
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

	CDBG("%s: open %s\n", __func__, filep->f_path.dentry->d_name.name);

	if (atomic_cmpxchg(&pmsm->opened, 0, 1) && once) {
		pr_err("%s: %s is already opened.\n",
			__func__,
			filep->f_path.dentry->d_name.name);
		return -EBUSY;
	}

	rc = nonseekable_open(inode, filep);
	if (rc < 0) {
		pr_err("%s: nonseekable_open error %d\n", __func__, rc);
		return rc;
	}

	rc = __msm_open(pmsm->sync, MSM_APPS_ID_PROP);
	if (rc < 0)
		return rc;

	filep->private_data = pmsm;

	CDBG("%s: rc %d\n", __func__, rc);
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

	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static int __msm_v4l2_control(struct msm_sync *sync,
		struct msm_ctrl_cmd *out)
{
	int rc = 0;

	struct msm_queue_cmd *qcmd = NULL, *rcmd = NULL;
	struct msm_ctrl_cmd *ctrl;
	struct msm_control_device_queue FIXME;

	/* wake up config thread, 4 is for V4L2 application */
	qcmd = kmalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!qcmd) {
		pr_err("%s: cannot allocate buffer\n", __func__);
		rc = -ENOMEM;
		goto end;
	}
	qcmd->type = MSM_CAM_Q_V4L2_REQ;
	qcmd->command = out;
	qcmd->on_heap = 1;

	rcmd = __msm_control(sync, &FIXME, qcmd, out->timeout_ms);
	if (IS_ERR(rcmd)) {
		rc = PTR_ERR(rcmd);
		goto end;
	}

	ctrl = (struct msm_ctrl_cmd *)(rcmd->command);
	/* FIXME: we should just set out->length = ctrl->length; */
	BUG_ON(out->length < ctrl->length);
	memcpy(out->value, ctrl->value, ctrl->length);

end:
	if (rcmd->on_heap)
		kfree(rcmd);
	CDBG("%s: rc %d\n", __func__, rc);
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
		pr_err("%s: error creating device: %d\n", __func__, rc);
		return rc;
	}

	cdev_init(&msm->cdev, fops);
	msm->cdev.owner = THIS_MODULE;

	rc = cdev_add(&msm->cdev, devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev: %d\n", __func__, rc);
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
		pr_err("%s: failed to initialize %s\n",
			__func__,
			sync->sdata->sensor_name);
		wake_lock_destroy(&sync->wake_lock);
		return rc;
	}

	sync->opencnt = 0;
	mutex_init(&sync->lock);
	CDBG("%s: initialized %s\n", __func__, sync->sdata->sensor_name);
	return rc;
}

static int msm_sync_destroy(struct msm_sync *sync)
{
	wake_lock_destroy(&sync->wake_lock);
	return 0;
}

static int msm_device_init(struct msm_device *pmsm,
		struct msm_sync *sync,
		int node)
{
	int dev_num = 3 * node;
	int rc = msm_setup_cdev(pmsm, node,
		MKDEV(MAJOR(msm_devno), dev_num),
		"control", &msm_fops_control);
	if (rc < 0) {
		pr_err("%s: error creating control node: %d\n", __func__, rc);
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 1, node,
		MKDEV(MAJOR(msm_devno), dev_num + 1),
		"config", &msm_fops_config);
	if (rc < 0) {
		pr_err("%s: error creating config node: %d\n", __func__, rc);
		msm_tear_down_cdev(pmsm, MKDEV(MAJOR(msm_devno),
				dev_num));
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 2, node,
		MKDEV(MAJOR(msm_devno), dev_num + 2),
		"frame", &msm_fops_frame);
	if (rc < 0) {
		pr_err("%s: error creating frame node: %d\n", __func__, rc);
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

int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
			struct msm_sensor_ctrl *))
{
	struct msm_device *pmsm = NULL;
	struct msm_sync *sync;
	int rc = -ENODEV;
	static int camera_node;

	if (camera_node >= MSM_MAX_CAMERA_SENSORS) {
		pr_err("%s: too many camera sensors\n", __func__);
		return rc;
	}

	if (!msm_class) {
		/* There are three device nodes per sensor */
		rc = alloc_chrdev_region(&msm_devno, 0,
				3 * MSM_MAX_CAMERA_SENSORS,
				"msm_camera");
		if (rc < 0) {
			pr_err("%s: failed to allocate chrdev: %d\n", __func__,
				rc);
			return rc;
		}

		msm_class = class_create(THIS_MODULE, "msm_camera");
		if (IS_ERR(msm_class)) {
			rc = PTR_ERR(msm_class);
			pr_err("%s: create device class failed: %d\n",
				__func__, rc);
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

	CDBG("%s: setting camera node %d\n", __func__, camera_node);
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
