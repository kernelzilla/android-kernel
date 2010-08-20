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

#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/msm_q6vdec.h>
#include <mach/dal.h>
#include <linux/file.h>

#define DALDEVICEID_VDEC_DEVICE         0x02000026
#define DALDEVICEID_VDEC_PORTNAME	"DAL_AQ_VID"

#define VDEC_INTERFACE_VERSION		0x00020000

#define MAJOR_MASK 0xFFFF0000
#define MINOR_MASK 0x0000FFFF

#define VDEC_GET_MAJOR_VERSION(version) (((version)&MAJOR_MASK)>>16)

#define VDEC_GET_MINOR_VERSION(version) ((version)&MINOR_MASK)

enum {
	VDEC_DALRPC_INITIALIZE = DALDEVICE_FIRST_DEVICE_API_IDX,
	VDEC_DALRPC_SETBUFFERS,
	VDEC_DALRPC_FREEBUFFERS,
	VDEC_DALRPC_QUEUE,
	VDEC_DALRPC_SIGEOFSTREAM,
	VDEC_DALRPC_FLUSH,
	VDEC_DALRPC_REUSEFRAMEBUFFER,
	VDEC_DALRPC_GETDECATTRIBUTES,
};

struct vdec_init_cfg {
	void *decode_done_evt;
	void *reuse_frame_evt;
	struct vdec_config cfg;
};

struct vdec_buffer_status {
	u32 data;
	u32 status;
};

#define VDEC_MSG_MAX 128

struct vdec_msg_list {
	struct list_head list;
	struct vdec_msg vdec_msg;
};

struct vdec_mem_info {
	u32 buf_type;
	u32 id;
	u32 phys_addr;
	u32 len;
	struct file *file;
};

struct vdec_mem_list {
	struct list_head list;
	struct vdec_mem_info mem;
};

struct vdec_data {
	void *vdec_handle;
	void *vdec_decode_done_evt_cb;
	void *vdec_reuse_frame_evt_cb;
	struct list_head vdec_msg_list_head;
	struct list_head vdec_msg_list_free;
	wait_queue_head_t vdec_msg_evt;
	struct mutex vdec_list_lock;
	struct list_head vdec_mem_list_head;
	struct mutex vdec_mem_list_lock;
	int mem_initialized;
	int running;
	int close_decode;
};

static struct class *driver_class;
static dev_t vdec_device_no;
static struct cdev vdec_cdev;
static int vdec_ref;
static DEFINE_MUTEX(vdec_ref_lock);

static inline int vdec_check_version(u32 client, u32 server)
{
	int ret = -EINVAL;
	if ((VDEC_GET_MAJOR_VERSION(client) == VDEC_GET_MAJOR_VERSION(server))
	    && (VDEC_GET_MINOR_VERSION(client) <=
		VDEC_GET_MINOR_VERSION(server)))
		ret = 0;
	return ret;
}

static int vdec_get_msg(struct vdec_data *vd, void *msg)
{
	struct vdec_msg_list *l;
	int ret = 0;

	if (!vd->running)
		return -EPERM;

	mutex_lock(&vd->vdec_list_lock);
	list_for_each_entry_reverse(l, &vd->vdec_msg_list_head, list) {
		if (copy_to_user(msg, &l->vdec_msg, sizeof(struct vdec_msg)))
			printk(KERN_ERR
			       "vdec_get_msg failed to copy_to_user!\n");
		list_del(&l->list);
		list_add(&l->list, &vd->vdec_msg_list_free);
		ret = 1;
		break;
	}
	mutex_unlock(&vd->vdec_list_lock);

	if (vd->close_decode)
		ret = 1;

	return ret;
}

static void vdec_put_msg(struct vdec_data *vd, struct vdec_msg *msg)
{
	struct vdec_msg_list *l;
	int found = 0;

	mutex_lock(&vd->vdec_list_lock);
	list_for_each_entry(l, &vd->vdec_msg_list_free, list) {
		memcpy(&l->vdec_msg, msg, sizeof(struct vdec_msg));
		list_del(&l->list);
		list_add(&l->list, &vd->vdec_msg_list_head);
		found = 1;
		break;
	}
	mutex_unlock(&vd->vdec_list_lock);

	if (found)
		wake_up(&vd->vdec_msg_evt);
	else
		printk(KERN_ERR "vdec_put_msg can't find free list!\n");
}

static struct vdec_mem_list *vdec_get_mem_from_list(struct vdec_data *vd,
						    u32 pmem_id, u32 buf_type)
{
	struct vdec_mem_list *l;
	int found = 0;

	mutex_lock(&vd->vdec_mem_list_lock);
	list_for_each_entry(l, &vd->vdec_mem_list_head, list) {
		if (l->mem.buf_type == buf_type && l->mem.id == pmem_id) {
			found = 1;
			break;
		}
	}
	mutex_unlock(&vd->vdec_mem_list_lock);

	if (found)
		return l;
	else
		return NULL;

}

static int vdec_initialize(struct vdec_data *vd, void *argp)
{
	struct vdec_config_sps vdec_cfg_sps;
	struct vdec_init_cfg vi_cfg;
	struct vdec_buf_req vdec_buf_req;
	struct u8 *header;
	int ret = 0;

	ret = copy_from_user(&vdec_cfg_sps,
			     &((struct vdec_init *)argp)->
			     sps_cfg, sizeof(vdec_cfg_sps));

	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	vi_cfg.decode_done_evt = vd->vdec_decode_done_evt_cb;
	vi_cfg.reuse_frame_evt = vd->vdec_reuse_frame_evt_cb;
	vi_cfg.cfg = vdec_cfg_sps.cfg;

	header = kmalloc(vdec_cfg_sps.seq.len, GFP_KERNEL);
	if (!header) {
		printk(KERN_ERR "%s: kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	ret = copy_from_user(header,
			     ((struct vdec_init *)argp)->
			     sps_cfg.seq.header, vdec_cfg_sps.seq.len);

	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		kfree(header);
		return ret;
	}

	ret =
	    dalrpc_fcn_13(VDEC_DALRPC_INITIALIZE,
			  vd->vdec_handle, (void *)&vi_cfg,
			  sizeof(vi_cfg),
			  (void *)vdec_cfg_sps.seq.header,
			  vdec_cfg_sps.seq.len,
			  (void *)&vdec_buf_req, sizeof(vdec_buf_req));

	kfree(header);

	if (ret)
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);
	else
		ret =
		    copy_to_user(((struct vdec_init *)argp)->
				 buf_req, &vdec_buf_req, sizeof(vdec_buf_req));

	vd->close_decode = 0;
	return ret;
}

static int vdec_setbuffers(struct vdec_data *vd, void *argp)
{
	struct vdec_buffer vmem;
	struct vdec_mem_list *l;
	unsigned long vstart;

	int ret = 0;

	vd->mem_initialized = 0;

	ret = copy_from_user(&vmem, argp, sizeof(vmem));
	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	l = kzalloc(sizeof(struct vdec_mem_list), GFP_KERNEL);
	if (!l) {
		printk(KERN_ERR "%s: kzalloc failed!\n", __func__);
		return -ENOMEM;
	}

	l->mem.id = vmem.pmem_id;
	l->mem.buf_type = vmem.buf.buf_type;

	ret = get_pmem_file(l->mem.id,
			    (unsigned long *)&l->mem.phys_addr,
			    &vstart,
			    (unsigned long *)&l->mem.len, &l->mem.file);

	if (ret) {
		printk(KERN_ERR "%s: get_pmem_fd failed\n", __func__);
		goto vdec_setbuffers_fail;
	}

	/* input buffers */
	if ((vmem.buf.region.offset + vmem.buf.region.size) > l->mem.len) {
		printk(KERN_ERR "%s: invalid input buffer offset!\n", __func__);
		ret = -EINVAL;
		goto vdec_setbuffers_fail;

	}
	vmem.buf.region.offset += l->mem.phys_addr;

	ret =
	    dalrpc_fcn_5(VDEC_DALRPC_SETBUFFERS,
			 vd->vdec_handle, (void *)&vmem.buf, sizeof(vmem.buf));
	if (ret) {
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);
		goto vdec_setbuffers_fail;
	}

	mutex_lock(&vd->vdec_mem_list_lock);

	list_add(&l->list, &vd->vdec_mem_list_head);

	mutex_unlock(&vd->vdec_mem_list_lock);

	vd->mem_initialized = 1;
	return ret;
vdec_setbuffers_fail:
	kfree(l);
	return ret;
}

static int vdec_queue(struct vdec_data *vd, void *argp)
{
	struct vdec_input_buf_info buf_info;
	struct vdec_mem_list *l;
	struct vdec_queue_status status;
	u32 pmem_id;
	int ret = 0;

	if (!vd->mem_initialized) {
		printk(KERN_ERR
		       "%s: memory is not being initialized!\n", __func__);
		return -EPERM;
	}

	ret = copy_from_user(&buf_info,
			     &((struct vdec_input_buf *)argp)->buffer,
			     sizeof(buf_info));
	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	ret = copy_from_user(&pmem_id,
			     &((struct vdec_input_buf *)argp)->pmem_id,
			     sizeof(u32));
	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	l = vdec_get_mem_from_list(vd, pmem_id, VDEC_BUFFER_TYPE_INPUT);

	if (NULL == l) {
		printk(KERN_ERR "%s: not able to find the buffer from list\n",
		       __func__);
		return -EPERM;
	}

	if ((buf_info.size + buf_info.offset) >= l->mem.len) {
		printk(KERN_ERR "%s: invalid queue buffer offset!\n", __func__);
		return -EINVAL;
	}

	buf_info.offset += l->mem.phys_addr;

	ret = dalrpc_fcn_8(VDEC_DALRPC_QUEUE, vd->vdec_handle,
			   (void *)&buf_info, sizeof(buf_info),
			   (void *)&status, sizeof(status));
	if (ret)
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);
	return ret;
}

static int vdec_reuse_framebuffer(struct vdec_data *vd, void *argp)
{
	u32 buf_id;
	int ret = 0;

	ret = copy_from_user(&buf_id, argp, sizeof(buf_id));
	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	ret =
	    dalrpc_fcn_0(VDEC_DALRPC_REUSEFRAMEBUFFER, vd->vdec_handle, buf_id);
	if (ret)
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);

	return ret;
}

static int vdec_flush(struct vdec_data *vd, void *argp)
{
	u32 flush_type;
	int ret = 0;

	ret = copy_from_user(&flush_type, argp, sizeof(flush_type));
	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	ret = dalrpc_fcn_0(VDEC_DALRPC_FLUSH, vd->vdec_handle, flush_type);
	if (ret) {
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int vdec_close(struct vdec_data *vd, void *argp)
{
	struct vdec_mem_list *l;
	int ret = 0;

	vd->close_decode = 1;
	wake_up(&vd->vdec_msg_evt);

	if (vd->mem_initialized) {
		list_for_each_entry(l, &vd->vdec_mem_list_head, list)
		    put_pmem_file(l->mem.file);
	}

	ret = daldevice_close(vd->vdec_handle);
	if (ret)
		printk(KERN_ERR
		       "%s: failed to close daldevice (%d)\n", __func__, ret);
	return ret;
}
static int vdec_getdecattributes(struct vdec_data *vd, void *argp)
{
	struct vdec_dec_attributes dec_attr;
	int ret = 0;

	ret = dalrpc_fcn_9(VDEC_DALRPC_GETDECATTRIBUTES, vd->vdec_handle,
			   (void *)&dec_attr, sizeof(dec_attr));
	if (ret)
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);
	else
		ret =
		    copy_to_user(((struct vdec_dec_attributes *)argp),
				 &dec_attr, sizeof(dec_attr));
	return ret;
}

static int vdec_freebuffers(struct vdec_data *vd, void *argp)
{
	struct vdec_buffer vmem;
	struct vdec_mem_list *l;

	int ret = 0;

	if (!vd->mem_initialized) {
		printk(KERN_ERR
		       "%s: memory is not being initialized!\n", __func__);
		return -EPERM;
	}

	ret = copy_from_user(&vmem, argp, sizeof(vmem));
	if (ret) {
		printk(KERN_ERR "%s: copy_from_user failed\n", __func__);
		return ret;
	}

	l = vdec_get_mem_from_list(vd, vmem.pmem_id, vmem.buf.buf_type);

	if (NULL == l) {
		printk(KERN_ERR "%s: not able to find the buffer from list\n",
		       __func__);
		return -EPERM;
	}

	/* input buffers */
	if ((vmem.buf.region.offset + vmem.buf.region.size) > l->mem.len) {
		printk(KERN_ERR "%s: invalid input buffer offset!\n", __func__);
		return -EINVAL;

	}
	vmem.buf.region.offset += l->mem.phys_addr;

	ret =
	    dalrpc_fcn_5(VDEC_DALRPC_FREEBUFFERS,
			 vd->vdec_handle, (void *)&vmem.buf, sizeof(vmem.buf));
	if (ret) {
		printk(KERN_ERR
		       "%s: remote function failed (%d)\n", __func__, ret);
	}

	return ret;
}
static long vdec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vdec_data *vd = file->private_data;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	if (!vd->running)
		return -EPERM;

	switch (cmd) {
	case VDEC_IOCTL_INITIALIZE:
		ret = vdec_initialize(vd, argp);
		break;

	case VDEC_IOCTL_SETBUFFERS:
		ret = vdec_setbuffers(vd, argp);
		break;

	case VDEC_IOCTL_QUEUE:
		ret = vdec_queue(vd, argp);
		break;

	case VDEC_IOCTL_REUSEFRAMEBUFFER:
		ret = vdec_reuse_framebuffer(vd, argp);
		break;

	case VDEC_IOCTL_FLUSH:
		ret = vdec_flush(vd, argp);
		break;

	case VDEC_IOCTL_EOS:
		ret = dalrpc_fcn_0(VDEC_DALRPC_SIGEOFSTREAM,
				   vd->vdec_handle, 0);
		if (ret)
			printk(KERN_ERR
			       "%s: remote function failed (%d)\n",
			       __func__, ret);
		break;

	case VDEC_IOCTL_GETMSG:
		wait_event_interruptible(vd->vdec_msg_evt,
					 vdec_get_msg(vd, argp));

		if (vd->close_decode)
			ret = -EINTR;
		break;

	case VDEC_IOCTL_CLOSE:
		ret = vdec_close(vd, argp);
		break;

	case VDEC_IOCTL_GETDECATTRIBUTES:
		ret = vdec_getdecattributes(vd, argp);

		if (ret)
			printk(KERN_ERR
			       "%s: remote function failed (%d)\n",
			       __func__, ret);
		break;

	case VDEC_IOCTL_FREEBUFFERS:
		ret = vdec_freebuffers(vd, argp);

		if (ret)
			printk(KERN_ERR
			       "%s: remote function failed (%d)\n",
			       __func__, ret);
		break;

	default:
		printk(KERN_ERR "%s: invalid ioctl!\n", __func__);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void vdec_dcdone_handler(void *context, uint32_t param, void *frame,
				uint32_t frame_size)
{
	struct vdec_msg msg;
	struct vdec_mem_list *l;
	int found = 0;
	struct vdec_data *vd = (struct vdec_data *)context;

	if (!vd->mem_initialized) {
		printk(KERN_ERR
		       "%s:memory not initialize but callback called!\n",
		       __func__);
		return;
	}
	if (frame_size != sizeof(struct vdec_frame_info)) {
		printk(KERN_ERR
		       "%s:frame_size is not same as vdec!\n", __func__);
		return;
	}

	memcpy(&msg.vfr_info, (struct vdec_frame_info *)frame,
	       sizeof(struct vdec_frame_info));

	if (msg.vfr_info.status == VDEC_FRAME_DECODE_OK) {
		mutex_lock(&vd->vdec_mem_list_lock);
		list_for_each_entry(l, &vd->vdec_mem_list_head, list) {
			if ((l->mem.buf_type == VDEC_BUFFER_TYPE_OUTPUT) &&
			    (msg.vfr_info.offset >= l->mem.phys_addr) &&
			    (msg.vfr_info.offset <
			     (l->mem.phys_addr + l->mem.len))) {
				found = 1;
				msg.vfr_info.offset -= l->mem.phys_addr;
				msg.vfr_info.data2 = l->mem.id;
				break;
			}
		}
		mutex_unlock(&vd->vdec_mem_list_lock);
	}

	if (found || (msg.vfr_info.status != VDEC_FRAME_DECODE_OK)) {
		msg.id = VDEC_MSG_FRAMEDONE;
		vdec_put_msg(vd, &msg);
	} else {
		printk(KERN_ERR
		       "%s: invalid phys addr = 0x%x\n",
		       __func__, msg.vfr_info.offset);
	}

}

static void vdec_reuseibuf_handler(void *context, uint32_t param, void *bufstat,
				   uint32_t bufstat_size)
{
	struct vdec_buffer_status *vdec_bufstat;
	struct vdec_msg msg;
	struct vdec_data *vd = (struct vdec_data *)context;

	if (bufstat_size != sizeof(struct vdec_buffer_status)) {
		printk(KERN_ERR
		       "%s:bufstat_size is not same as vdec_bufstat!\n",
		       __func__);
		return;
	}

	vdec_bufstat = (struct vdec_buffer_status *)bufstat;
	msg.id = VDEC_MSG_REUSEINPUTBUFFER;
	msg.buf_id = vdec_bufstat->data;
	vdec_put_msg(vd, &msg);
}

static int vdec_open(struct inode *inode, struct file *file)
{
	int ret;
	int i;
	struct vdec_msg_list *l;
	struct vdec_data *vd;
	struct daldevice_info_t deviceinfo;

	mutex_lock(&vdec_ref_lock);
	if (vdec_ref > 0) {
		printk(KERN_ERR "%s: Instance already running\n", __func__);
		mutex_unlock(&vdec_ref_lock);
		return -EBUSY;
	}
	vdec_ref++;
	mutex_unlock(&vdec_ref_lock);

	vd = kmalloc(sizeof(struct vdec_data), GFP_KERNEL);
	if (!vd) {
		printk(KERN_ERR "%s: kmalloc failed\n", __func__);
		ret = -ENOMEM;
		goto vdec_open_err_handle_vd;
	}
	file->private_data = vd;

	vd->mem_initialized = 0;
	INIT_LIST_HEAD(&vd->vdec_msg_list_head);
	INIT_LIST_HEAD(&vd->vdec_msg_list_free);
	INIT_LIST_HEAD(&vd->vdec_mem_list_head);
	init_waitqueue_head(&vd->vdec_msg_evt);

	mutex_init(&vd->vdec_list_lock);
	mutex_init(&vd->vdec_mem_list_lock);
	for (i = 0; i < VDEC_MSG_MAX; i++) {
		l = kzalloc(sizeof(struct vdec_msg_list), GFP_KERNEL);
		if (!l) {
			printk(KERN_ERR "%s: kzalloc failed!\n", __func__);
			ret = -ENOMEM;
			goto vdec_open_err_handle_list;
		}
		list_add(&l->list, &vd->vdec_msg_list_free);
	}

	ret = daldevice_attach(DALDEVICEID_VDEC_DEVICE,
			       DALDEVICEID_VDEC_PORTNAME,
			       DALRPC_DEST_QDSP, &vd->vdec_handle);
	if (ret) {
		printk(KERN_ERR "%s: failed to attach (%d)\n", __func__, ret);
		ret = -EIO;
		goto vdec_open_err_handle_list;
	}

	ret = daldevice_info(vd->vdec_handle, &deviceinfo, sizeof(deviceinfo));

	if (ret) {
		printk(KERN_ERR "%s: failed to get the version info (%d)\n",
		       __func__, ret);
		ret = -EIO;
		goto vdec_open_err_handle_daldetach;
	}

	if (vdec_check_version(VDEC_INTERFACE_VERSION, deviceinfo.version)) {
		printk(KERN_ERR "%s: failed version mismatch \n", __func__);
		ret = -EIO;
		goto vdec_open_err_handle_daldetach;
	}

	vd->vdec_reuse_frame_evt_cb =
	    dalrpc_alloc_cb(vd->vdec_handle, vdec_reuseibuf_handler, vd);
	if (vd->vdec_reuse_frame_evt_cb == NULL) {
		printk(KERN_ERR "%s: dalrpc_alloc_cb failed\n", __func__);
		ret = -ENOMEM;
		goto vdec_open_err_handle_daldetach;

	}

	vd->vdec_decode_done_evt_cb =
	    dalrpc_alloc_cb(vd->vdec_handle, vdec_dcdone_handler, vd);
	if (vd->vdec_decode_done_evt_cb == NULL) {
		printk(KERN_ERR "%s: dalrpc_alloc_cb failed\n", __func__);
		ret = -ENOMEM;
		goto vdec_open_err_handle_deevent;

	}

	vd->running = 1;

	return 0;

vdec_open_err_handle_deevent:
	dalrpc_dealloc_cb(vd->vdec_handle, vd->vdec_reuse_frame_evt_cb);
vdec_open_err_handle_daldetach:
	daldevice_detach(vd->vdec_handle);
vdec_open_err_handle_list:
	{
		struct vdec_msg_list *l;
		list_for_each_entry(l, &vd->vdec_msg_list_free, list) {
			list_del(&l->list);
			kfree(l);
		}
	}
vdec_open_err_handle_vd:
	mutex_lock(&vdec_ref_lock);
	vdec_ref--;
	mutex_unlock(&vdec_ref_lock);
	kfree(vd);
	return ret;
}

static int vdec_release(struct inode *inode, struct file *file)
{
	int ret;
	struct vdec_msg_list *l, *n;
	struct vdec_data *vd = file->private_data;

	vd->running = 0;
	wake_up_all(&vd->vdec_msg_evt);

	dalrpc_dealloc_cb(vd->vdec_handle, vd->vdec_decode_done_evt_cb);
	dalrpc_dealloc_cb(vd->vdec_handle, vd->vdec_reuse_frame_evt_cb);

	ret = daldevice_detach(vd->vdec_handle);
	if (ret)
		printk(KERN_INFO "%s: failed to detach (%d)\n", __func__, ret);

	list_for_each_entry_safe(l, n, &vd->vdec_msg_list_free, list) {
		list_del(&l->list);
		kfree(l);
	}

	list_for_each_entry_safe(l, n, &vd->vdec_msg_list_head, list) {
		list_del(&l->list);
		kfree(l);
	}

	list_for_each_entry_safe(l, n, &vd->vdec_mem_list_head, list) {
		list_del(&l->list);
		kfree(l);
	}

	mutex_lock(&vdec_ref_lock);
	BUG_ON(vdec_ref <= 0);
	vdec_ref--;
	mutex_unlock(&vdec_ref_lock);
	kfree(vd);
	return 0;
}

static const struct file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.open = vdec_open,
	.release = vdec_release,
	.unlocked_ioctl = vdec_ioctl,
};

static int __init vdec_init(void)
{
	struct device *class_dev;
	int rc = 0;

	rc = alloc_chrdev_region(&vdec_device_no, 0, 1, "vdec");
	if (rc < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region failed %d\n",
		       __func__, rc);
		return rc;
	}

	driver_class = class_create(THIS_MODULE, "vdec");
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		printk(KERN_ERR "%s: class_create failed %d\n", __func__, rc);
		goto vdec_init_err_unregister_chrdev_region;
	}
	class_dev = device_create(driver_class, NULL,
				  vdec_device_no, NULL, "vdec");
	if (!class_dev) {
		printk(KERN_ERR "%s: class_device_create failed %d\n",
		       __func__, rc);
		rc = -ENOMEM;
		goto vdec_init_err_class_destroy;
	}

	cdev_init(&vdec_cdev, &vdec_fops);
	vdec_cdev.owner = THIS_MODULE;
	rc = cdev_add(&vdec_cdev, MKDEV(MAJOR(vdec_device_no), 0), 1);

	if (rc < 0) {
		printk(KERN_ERR "%s: cdev_add failed %d\n", __func__, rc);
		goto vdec_init_err_class_device_destroy;
	}

	return 0;

vdec_init_err_class_device_destroy:
	device_destroy(driver_class, vdec_device_no);
vdec_init_err_class_destroy:
	class_destroy(driver_class);
vdec_init_err_unregister_chrdev_region:
	unregister_chrdev_region(vdec_device_no, 1);
	return rc;
}

static void __exit vdec_exit(void)
{
	device_destroy(driver_class, vdec_device_no);
	class_destroy(driver_class);
	unregister_chrdev_region(vdec_device_no, 1);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("video decoder driver for QSD platform");
MODULE_VERSION("2.00");

module_init(vdec_init);
module_exit(vdec_exit);
