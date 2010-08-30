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

struct msm_device_t **msm_camera;

static struct class *msm_class;
static dev_t msm_devno;

uint8_t disable_lens_move = 0;

static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info_t *info, struct file *file, unsigned long paddr,
	unsigned long len, int fd)
{
	struct msm_pmem_region *region =
		kmalloc(sizeof(*region), GFP_KERNEL);

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
	region->fd = fd;
	region->active = info->active;

	hlist_add_head(&(region->list), ptype);

	return 0;
}

static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	enum msm_pmem_t type, struct msm_pmem_region *reg, uint8_t maxcount)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node;

	uint8_t rc = 0;

	regptr = reg;

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

	return rc;
}

static unsigned long msm_pmem_frame_ptov_lookup(unsigned long pyaddr,
	unsigned long pcbcraddr, uint32_t *yoff, uint32_t *cbcroff, int *fd,
	struct msm_device_t	*msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;
	unsigned long rc = 0;

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

	return 0;
}

static unsigned long msm_pmem_stats_ptov_lookup(unsigned long addr, int *fd,
	struct msm_device_t *msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;
	unsigned long rc = 0;

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

	return 0;
}

static void msm_pmem_frame_vtop_lookup(unsigned long buffer,
	uint32_t yoff, uint32_t cbcroff, int fd, unsigned long *phyaddr,
	struct msm_device_t *msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;

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
}

static void msm_pmem_stats_vtop_lookup(unsigned long buffer,
	int fd, unsigned long *phyaddr, struct msm_device_t *msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;

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
}

static long msm_pmem_table_del_proc(struct msm_pmem_info_t *pinfo,
	struct msm_device_t *msm)
{
	long rc = 0;
	struct msm_pmem_region *region;
	struct hlist_node *node;
	struct hlist_node *n;

	mutex_lock(&msm->msm_sem);
	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
	case MSM_PMEM_THUMBAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
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
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
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
		break;

	default:
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&msm->msm_sem);

	return rc;
}

static long msm_pmem_table_del(void __user *arg,
	struct msm_device_t *msm)
{
	struct msm_pmem_info_t info;

	if (copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;

	return msm_pmem_table_del_proc(&info, msm);
}

static long msm_get_frame_proc(struct msm_frame_t *frame,
	struct msm_device_t	*msm)
{
	unsigned long flags;
	long rc = 0;

	struct msm_queue_cmd_t *qcmd = NULL;
	struct msm_vfe_phy_info *pphy;

	spin_lock_irqsave(&msm->sync.prev_frame_q_lock, flags);

	if (!list_empty(&msm->sync.prev_frame_q)) {
		qcmd = list_first_entry(&msm->sync.prev_frame_q,
			struct msm_queue_cmd_t, list);
		list_del(&qcmd->list);
	}

	spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock, flags);

	if (!qcmd)
		return -EAGAIN;

	pphy = (struct msm_vfe_phy_info *)(qcmd->command);

	frame->buffer =
		msm_pmem_frame_ptov_lookup(pphy->y_phy,
			pphy->cbcr_phy, &(frame->y_off),
			&(frame->cbcr_off), &(frame->fd), msm);

	CVBS("get_fr_proc: y= 0x%x, cbcr= 0x%x, qcmd= 0x%x, virt_addr= 0x%x\n",
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);

	kfree(qcmd->command);
	kfree(qcmd);
	return rc;
}

static long msm_get_frame(void __user *arg,
	struct msm_device_t	*msm)
{
	long rc = 0;
	struct msm_frame_t frame;

	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame_t)))
		return -EFAULT;

	rc = msm_get_frame_proc(&frame, msm);
	if (rc < 0)
		return rc;
	if (msm->croplen) {
		if (frame.croplen > msm->croplen)
			return -EINVAL;

		if (copy_to_user((void *)frame.cropinfo,
				msm->cropinfo,
				msm->croplen))
		return -EFAULT;
	}


	if (copy_to_user((void *)arg,
				&frame, sizeof(struct msm_frame_t)))
		rc = -EFAULT;

	CVBS("Got frame!!!\n");

	return rc;
}

static long msm_enable_vfe(void __user *arg,
	struct msm_device_t *msm)
{
	long rc = 0;
	struct camera_enable_cmd_t *cfg;
	struct camera_enable_cmd_t cfg_t;

	if (copy_from_user(
				&cfg_t,
				arg,
				sizeof(struct camera_enable_cmd_t)))
		return -EFAULT;

	cfg = kmalloc(sizeof(struct camera_enable_cmd_t),
					GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	cfg->name = kmalloc(cfg_t.length, GFP_KERNEL);

	if (!(cfg->name)) {
		kfree(cfg);
		return -ENOMEM;
	}

	if (copy_from_user(cfg->name, (void *)cfg_t.name, cfg_t.length))
		return -EFAULT;

	if (msm->vfefn.vfe_enable)
		rc = msm->vfefn.vfe_enable(cfg);

	CDBG("msm_enable_vfe:returned rc = %ld\n", rc);

	kfree(cfg);
	return rc;
}

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

	struct msm_ctrl_cmd_t ctrlcmd_t;
	struct msm_ctrl_cmd_t *ctrlcmd;
	struct msm_queue_cmd_t *qcmd = NULL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd_t))) {
		rc = -EFAULT;
		goto end;
	}

	ctrlcmd = kmalloc(sizeof(struct msm_ctrl_cmd_t), GFP_ATOMIC);
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

    CDBG("%s type = %d\n", __func__, ctrlcmd_t.type);

	/* wait for config status */
	timeout = (int)ctrlcmd_t.timeout_ms;
	CDBG("%s timeout = %d\n", __func__, timeout);
	if (timeout > 0) {
		rc = wait_event_timeout(msm->sync.ctrl_status_wait,
					msm_ctrl_stats_pending(msm),
					msecs_to_jiffies(timeout));

		CDBG("msm_control: rc = %ld\n", rc);

		if (rc == 0) {
			CDBG("msm_control: timed out\n");
			rc = -ETIMEDOUT;
			goto timeout;
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

		CDBG("msm_control: length = %d\n",
			((struct msm_ctrl_cmd_t *)(qcmd->command))->length);
		ctrlcmd_t.type =
			((struct msm_ctrl_cmd_t *)(qcmd->command))->type;

		ctrlcmd_t.length =
			((struct msm_ctrl_cmd_t *)(qcmd->command))->length;

		ctrlcmd_t.status =
			((struct msm_ctrl_cmd_t *)(qcmd->command))->status;

		if (ctrlcmd_t.length > 0) {
			if (copy_to_user(ctrlcmd_t.value,
			((struct msm_ctrl_cmd_t *)(qcmd->command))->value,
			((struct msm_ctrl_cmd_t *)(qcmd->command))->length)) {

				CDBG("copy_to_user value failed!\n");
				rc = -EFAULT;
				goto end;
			}

			kfree(((struct msm_ctrl_cmd_t *)
				(qcmd->command))->value);
		}

		if (copy_to_user((void *)arg,
				&ctrlcmd_t,
				sizeof(struct msm_ctrl_cmd_t))) {
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

static int msm_stats_pending(struct msm_device_t *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd_t *qcmd = NULL;

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock,
		flags);

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

	CVBS("msm_stats_pending, tyes = %d\n", yes);
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
	struct msm_ctrl_cmd_t  *ctrl = NULL;
	struct msm_vfe_resp_t  *data = NULL;
	struct msm_stats_buf_t stats;

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
			CDBG("msm_get_stats, timeout=%d\n",timeout);
			return -ETIMEDOUT;
		}
	} else {
		rc = wait_event_interruptible(msm->sync.msg_event_wait,
			msm_stats_pending(msm));
	}

	if (rc < 0) {
		CDBG("msm_get_stats, rc = %ld\n", rc);
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

	CVBS("=== received from DSP === %d\n", qcmd->type);

	switch (qcmd->type) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		data = (struct msm_vfe_resp_t *)(qcmd->command);

		/* adsp event and message */
		se.resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		se.stats_event.type   = data->evt_msg.type;
		se.stats_event.msg_id = data->evt_msg.msg_id;
		se.stats_event.len    = data->evt_msg.len;

		CVBS("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CVBS("length = %d\n", se.stats_event.len);
		CVBS("msg_id = %d\n", se.stats_event.msg_id);

		if ((data->type == VFE_MSG_STATS_AF) ||
				(data->type == VFE_MSG_STATS_WE)) {

			stats.buffer =
			msm_pmem_stats_ptov_lookup(data->phy.sbuf_phy,
				&(stats.fd), msm);

			if (copy_to_user((void *)(se.stats_event.data),
				&stats, sizeof(struct msm_stats_buf_t))) {

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
			struct msm_postproc_t buf;
			struct msm_pmem_region region;
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
          CDBG("%s: Copy_to_user: buf=0x%x fd=%d y_o=%d c_o=%d\n", __func__,
            buf.fmain.buffer, buf.fmain.fd, buf.fmain.y_off, buf.fmain.cbcr_off);
			if (copy_to_user((void *)(se.stats_event.data),
						&(buf.fmain),
						sizeof(struct msm_frame_t)))
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
			struct msm_postproc_t buf;
			struct msm_pmem_region region;

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
					&buf, sizeof(struct msm_postproc_t))) {

					rc = -EFAULT;
					goto failure;
				}
			}

			CDBG("SNAPSHOT copy_to_user!\n");
		}
		break;

	case MSM_CAM_Q_CTRL:{
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd_t *)(qcmd->command);

		CVBS("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CVBS("length = %d\n", ctrl->length);

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
		ctrl = (struct msm_ctrl_cmd_t *)(qcmd->command);

		CVBS("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CVBS("length = %d\n", ctrl->length);

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
        CDBG("msm_get_stats: end rc = %ld\n", rc);
    } else {
    	CVBS("msm_get_stats: end rc = %ld\n", rc);
    }
	return rc;
}

static long msm_ctrl_cmd_done(void __user *arg,
	struct msm_device_t *msm)
{
	unsigned long flags;
	long rc = 0;

	struct msm_ctrl_cmd_t ctrlcmd_t;
	struct msm_ctrl_cmd_t *ctrlcmd;
	struct msm_queue_cmd_t *qcmd = NULL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd_t)))
		return -EFAULT;

	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd_t), GFP_ATOMIC);
	if (!ctrlcmd) {
		rc = -ENOMEM;
		goto end;
	}

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
	} else
		ctrlcmd->value = NULL;

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
end:
	CDBG("msm_ctrl_cmd_done: end rc = %ld\n", rc);
	if (rc == 0) {
		/* wake up control thread */
		spin_lock_irqsave(&msm->sync.ctrl_status_lock, flags);
		list_add_tail(&qcmd->list, &msm->sync.ctrl_status_queue);
		spin_unlock_irqrestore(&msm->sync.ctrl_status_lock, flags);
		wake_up(&msm->sync.ctrl_status_wait);
	}

	return rc;
}

static long msm_config_vfe(void __user *arg,
	struct msm_device_t	*msm)
{
	struct msm_vfe_cfg_cmd_t cfgcmd_t;
	struct msm_pmem_region region[8];
	struct axidata_t axi_data;
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

static long msm_frame_axi_cfg(struct msm_vfe_cfg_cmd_t *cfgcmd_t,
	struct msm_device_t *msm)
{
	long rc = 0;
	struct axidata_t axi_data;
	struct msm_pmem_region region[8];
	enum msm_pmem_t mtype;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd_t->cmd_type) {
	case CMD_AXI_CFG_OUT1:
		mtype = MSM_PMEM_OUTPUT1;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
		break;

	case CMD_AXI_CFG_OUT2:
		mtype = MSM_PMEM_OUTPUT2;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
		break;

	case CMD_AXI_CFG_SNAP_O1_AND_O2:
		mtype = MSM_PMEM_THUMBAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);

		mtype = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[axi_data.bufnum1], 8);
		break;

	case CMD_RAW_PICT_AXI_CFG:
		mtype = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
		break;

	default:
		break;
	}

	axi_data.region = &region[0];

	/* send the AXI configuration command to driver */
	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(cfgcmd_t, &axi_data);

	return rc;
}

static long msm_get_sensor_info(void __user *arg,
	struct msm_device_t *msm)
{
	long rc = 0;
	struct msm_camsensor_info_t info;
	struct msm_camera_device_platform_data *pdata;

	if (copy_from_user(&info,
			arg,
			sizeof(struct msm_camsensor_info_t)))
		return -EFAULT;

	pdata = msm->pdev->dev.platform_data;
	CDBG("sensor_name %s\n", pdata->sinfo[msm->sidx].sensor_name);

	memcpy(&info.name[0],
		pdata->sinfo[msm->sidx].sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled =
		(pdata->sinfo[msm->sidx].flash_type != MSM_CAMERA_FLASH_NONE);

	/* copy back to user space */
	if (copy_to_user((void *)arg,
			&info,
			sizeof(struct msm_camsensor_info_t))) {

		CDBG("get senso copy to user failed!\n");
		rc = -EFAULT;
	}

	return rc;
}

static long msm_put_frame_buf_proc(struct msm_frame_t *pb,
	struct msm_device_t *msm)
{
	unsigned long pphy;
	struct msm_vfe_cfg_cmd_t cfgcmd_t;

	long rc = 0;

	msm_pmem_frame_vtop_lookup(pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd, &pphy, msm);

	CVBS("rel: vaddr = 0x%lx, paddr = 0x%lx\n",
		pb->buffer, pphy);

	if (pphy != 0) {

		cfgcmd_t.cmd_type = CMD_FRAME_BUF_RELEASE;
		cfgcmd_t.value    = (void *)pb;

		if (msm->vfefn.vfe_config)
			rc =
				msm->vfefn.vfe_config(&cfgcmd_t, &pphy);
	} else
		rc = -EFAULT;

	return rc;
}

static long msm_put_frame_buffer(void __user *arg,
	struct msm_device_t *msm)
{
	struct msm_frame_t buf_t;

	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame_t)))
		return -EFAULT;

	return msm_put_frame_buf_proc(&buf_t, msm);
}

static long msm_register_pmem_proc(struct msm_pmem_info_t *pinfo,
	struct msm_device_t *msm)
{
	unsigned long paddr, len, rc = 0;
	struct file   *file;
	unsigned long vstart;

	mutex_lock(&msm->msm_sem);

	CVBS("Here1 ==> reg: type = %d, paddr = 0x%lx, vaddr = 0x%lx\n",
		pinfo->type, paddr, (unsigned long)pinfo->vaddr);

	get_pmem_file(pinfo->fd, &paddr, &vstart, &len, &file);

	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
	case MSM_PMEM_THUMBAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
		rc = msm_pmem_table_add(&msm->sync.frame,
			pinfo, file, paddr, len, pinfo->fd);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		rc = msm_pmem_table_add(&msm->sync.stats,
			pinfo, file, paddr, len, pinfo->fd);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&msm->msm_sem);
	return rc;
}

static long msm_register_pmem(void __user *arg,
	struct msm_device_t *msm)
{
	struct msm_pmem_info_t info;

	if (copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;

	return msm_register_pmem_proc(&info, msm);
}

static long msm_stats_axi_cfg(struct msm_vfe_cfg_cmd_t *cfgcmd_t,
	struct msm_device_t *msm)
{
	long rc = 0;
	struct axidata_t axi_data;

	struct msm_pmem_region region[3];
	enum msm_pmem_t mtype = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

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

	return rc;
}

static long msm_put_stats_buffer(void __user *arg,
	struct msm_device_t *msm)
{
	long rc = 0;

	struct msm_stats_buf_t buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd_t cfgcmd_t;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf_t)))
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

static long msm_axi_config(void __user *arg,
	struct msm_device_t *msm)
{
	long rc = 0;
	struct msm_vfe_cfg_cmd_t cfgcmd_t;

	if (copy_from_user(&cfgcmd_t, arg, sizeof(cfgcmd_t)))
		return -EFAULT;

	switch (cfgcmd_t.cmd_type) {
	case CMD_AXI_CFG_OUT1:
	case CMD_AXI_CFG_OUT2:
	case CMD_AXI_CFG_SNAP_O1_AND_O2:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(&cfgcmd_t, msm);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(&cfgcmd_t, msm);

	default:
		rc = -EFAULT;
	}

	return rc;
}

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

static long msm_get_pict_proc(struct msm_ctrl_cmd_t *ctrl,
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
		((struct msm_ctrl_cmd_t *)(qcmd->command))->type;

		ctrl->status =
		((struct msm_ctrl_cmd_t *)(qcmd->command))->status;

		kfree(qcmd->command);
	} else {
		ctrl->type = 0xFFFF;
		ctrl->status = 0xFFFF;
	}

	kfree(qcmd);

	return rc;
}

static long msm_get_pic(void __user *arg,
	struct msm_device_t *msm)
{
	struct msm_ctrl_cmd_t ctrlcmd_t;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd_t)))
		return -EFAULT;

	if (msm_get_pict_proc(&ctrlcmd_t, msm) < 0)
		return -EFAULT;
	if (msm->croplen) {
		if (ctrlcmd_t.length < msm->croplen)
			return -EINVAL;

		if (copy_to_user(ctrlcmd_t.value,
				msm->cropinfo,
				msm->croplen))
			return -EINVAL;
	}


	if (copy_to_user((void *)arg,
		&ctrlcmd_t,
		sizeof(struct msm_ctrl_cmd_t)))
		return -EFAULT;
	return 0;
}

static long msm_set_crop(void __user *arg,
	struct msm_device_t *msm)
{
	struct crop_info_t crop;

	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info_t)))
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

static long msm_pict_pp_done(void __user *arg,
	struct msm_device_t *msm)
{
	struct msm_ctrl_cmd_t ctrlcmd_t;
	struct msm_ctrl_cmd_t *ctrlcmd = NULL;
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
				sizeof(struct msm_ctrl_cmd_t))) {
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

	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd_t), GFP_ATOMIC);
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
			arg, sizeof(struct msm_ctrl_cmd_t))) {
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
			sizeof(struct msm_ctrl_cmd_t))) {
		CDBG("msm_af_control: copy_to_user ctrlcmd failed!\n");
		rc = -EFAULT;
	}

end:
    CDBG("msm_af_control: end rc = %ld\n", rc);

	return rc;
}
static long msm_af_control_done(void __user *arg,
	struct msm_device_t *msm)
{
	unsigned long flags;
	long rc = 0;
	rc = copy_from_user(&msm->sync.af_status,
		arg, sizeof(struct msm_ctrl_cmd_t));
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	msm->sync.af_flag = (rc == 0 ? 1 : -1);
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	wake_up(&msm->sync.af_status_wait);
	return rc;
}
static long msm_ioctl(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct msm_device_t *pmsm = filep->private_data;

	CVBS("!!! msm_ioctl !!!, cmd = %d\n", _IOC_NR(cmd));

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

	case MSM_CAM_IOCTL_RELEASE_FRAMEE_BUFFER:
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
		enum msm_camera_led_state_t led_state;
		if (copy_from_user(&led_state, argp, sizeof(led_state)))
			return -EFAULT;
		return flash_set_led_state(led_state);
	}

        case MSM_CAM_IOCTL_DISABLE_LENS_MOVE:
                 printk("CAM_TCMD: lens move disabling.");
                disable_lens_move = 1;
                return 0;

        case MSM_CAM_IOCTL_ENABLE_LENS_MOVE:
                 printk("CAM_TCMD: lens move enabling.");
                disable_lens_move = 0;
                return 0;

	case MSM_CAM_IOCTL_AF_CTRL:
		return msm_af_control(argp, pmsm);

	case MSM_CAM_IOCTL_AF_CTRL_DONE:
		return msm_af_control_done(argp, pmsm);

	default:
		break;
	}

	return -EINVAL;
}

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

#ifndef CONFIG_MACH_MOT
			mutex_lock(&msm->pict_pp_lock);
#endif
			pp = msm->pict_pp;
#ifndef CONFIG_MACH_MOT
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

			CVBS("qcmd_frame= 0x%x phy_y= 0x%x, phy_cbcr= 0x%x\n",
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

#ifndef CONFIG_MACH_MOT
			mutex_lock(&msm->pict_pp_lock);
#endif
			pp = msm->pict_pp;
#ifndef CONFIG_MACH_MOT
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

static long msm_control_proc(struct msm_ctrl_cmd_t *ctrlcmd,
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
		((struct msm_ctrl_cmd_t *)(rcmd->command))->value,
		((struct msm_ctrl_cmd_t *)(rcmd->command))->length);

	if (((struct msm_ctrl_cmd_t *)(rcmd->command))->length > 0)
		kfree(((struct msm_ctrl_cmd_t *)
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
