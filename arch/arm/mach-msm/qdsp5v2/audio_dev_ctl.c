/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/msm_audio.h>
#include <asm/uaccess.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>

#define AUDIO_DEV_CTL_MAX_DEV 16

struct audio_dev_ctrl_state {
	struct mutex lock;
	struct msm_snddev_info *devs[AUDIO_DEV_CTL_MAX_DEV];
	u32 num_dev;
	u32 opened;
	struct msm_snddev_info *voice_rx_dev;
	struct msm_snddev_info *voice_tx_dev;
};

static struct audio_dev_ctrl_state audio_dev_ctrl;

void msm_snddev_register(struct msm_snddev_info *dev_info)
{
	mutex_lock(&audio_dev_ctrl.lock);
	if (audio_dev_ctrl.num_dev < AUDIO_DEV_CTL_MAX_DEV) {
		audio_dev_ctrl.devs[audio_dev_ctrl.num_dev] = dev_info;
		audio_dev_ctrl.num_dev++;
	} else
		pr_err("%s: device registry max out\n", __func__);
	mutex_unlock(&audio_dev_ctrl.lock);
}

int msm_get_voc_route(u32 *rx_id, u32 *tx_id)
{
	int rc = 0;

	if (!rx_id || !tx_id)
		return -EINVAL;

	mutex_lock(&audio_dev_ctrl.lock);
	if (!audio_dev_ctrl.voice_rx_dev || !audio_dev_ctrl.voice_tx_dev) {
		rc = -ENODEV;
		goto error;
	}

	*rx_id = audio_dev_ctrl.voice_rx_dev->acdb_id;
	*tx_id = audio_dev_ctrl.voice_tx_dev->acdb_id;
error:
	mutex_unlock(&audio_dev_ctrl.lock);
	return rc;
}
EXPORT_SYMBOL(msm_get_voc_route);

static struct msm_snddev_info *audio_dev_ctrl_find_dev(
	struct audio_dev_ctrl_state *dev_ctrl, u32 dev_id)
{
	struct msm_snddev_info *info;

	if ((dev_ctrl->num_dev - 1) < dev_id) {
		info = ERR_PTR(-ENODEV);
		goto error;
	}

	info = dev_ctrl->devs[dev_id];
error:
	return info;

}

static int audio_dev_ctrl_get_devices(struct audio_dev_ctrl_state *dev_ctrl,
				      void __user *arg)
{
	int rc = 0;
	u32 index;
	struct msm_snd_device_list work_list;
	struct msm_snd_device_info *work_tbl;

	if (copy_from_user(&work_list, arg, sizeof(work_list))) {
		rc = -EFAULT;
		goto error;
	}

	if (work_list.num_dev > dev_ctrl->num_dev) {
		rc = -EINVAL;
		goto error;
	}

	work_tbl = kmalloc(work_list.num_dev *
		sizeof(struct msm_snd_device_info), GFP_KERNEL);
	if (!work_tbl) {
		rc = -ENOMEM;
		goto error;
	}

	for (index = 0; index < dev_ctrl->num_dev; index++) {
		work_tbl[index].dev_id = index;
		work_tbl[index].dev_cap = dev_ctrl->devs[index]->capability;
		strlcpy(work_tbl[index].dev_name, dev_ctrl->devs[index]->name,
		64);
	}

	if (copy_to_user((void *) (work_list.list), work_tbl,
		 work_list.num_dev * sizeof(struct msm_snd_device_info)))
		rc = -EFAULT;
	kfree(work_tbl);
error:
	return rc;
}

static int audio_dev_ctrl_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct audio_dev_ctrl_state *dev_ctrl = file->private_data;

	mutex_lock(&audio_dev_ctrl.lock);
	switch (cmd) {
	case AUDIO_GET_NUM_SND_DEVICE:
		rc = put_user(dev_ctrl->num_dev, (uint32_t __user *) arg);
		break;
	case AUDIO_GET_SND_DEVICES:
		rc = audio_dev_ctrl_get_devices(dev_ctrl, (void __user *) arg);
		break;
	case AUDIO_ENABLE_SND_DEVICE: {
		struct msm_snddev_info *dev_info;
		u32 dev_id;

		if (get_user(dev_id, (u32 __user *) arg)) {
			rc = -EFAULT;
			break;
		}
		dev_info = audio_dev_ctrl_find_dev(dev_ctrl, dev_id);
		if (IS_ERR(dev_info))
			rc = PTR_ERR(dev_info);
		else
			rc = dev_info->dev_ops.open(dev_info);
		break;

	}

	case AUDIO_DISABLE_SND_DEVICE: {
		struct msm_snddev_info *dev_info;
		u32 dev_id;

		if (get_user(dev_id, (u32 __user *) arg)) {
			rc = -EFAULT;
			break;
		}
		dev_info = audio_dev_ctrl_find_dev(dev_ctrl, dev_id);
		if (IS_ERR(dev_info))
			rc = PTR_ERR(dev_info);
		else
			rc = dev_info->dev_ops.close(dev_info);
		break;
	}

	case AUDIO_ROUTE_STREAM: {
		struct msm_audio_route_config route_cfg;
		struct msm_snddev_info *dev_info;

		if (copy_from_user(&route_cfg, (void __user *) arg,
			sizeof(struct msm_audio_route_config))) {
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: route cfg %d %d type\n", __func__,
		route_cfg.dev_id, route_cfg.stream_type);
		dev_info = audio_dev_ctrl_find_dev(dev_ctrl,
			   route_cfg.dev_id);
		if (IS_ERR(dev_info)) {
			pr_err("%s: pass invalid dev_id\n", __func__);
			rc = PTR_ERR(dev_info);
			break;
		}

		switch (route_cfg.stream_type) {

		case AUDIO_ROUTE_STREAM_VOICE_RX:
			if (!(dev_info->capability & SNDDEV_CAP_RX) |
			    !(dev_info->capability & SNDDEV_CAP_VOICE)) {
				rc = -EINVAL;
				break;
			}
			dev_ctrl->voice_rx_dev = dev_info;
			break;
		case AUDIO_ROUTE_STREAM_VOICE_TX:
			if (!(dev_info->capability & SNDDEV_CAP_TX) |
			    !(dev_info->capability & SNDDEV_CAP_VOICE)) {
				rc = -EINVAL;
				break;
			}
			dev_ctrl->voice_tx_dev = dev_info;
			break;
		}
		break;
	}

	default:
		rc = -EINVAL;
	}
	mutex_unlock(&audio_dev_ctrl.lock);
	return rc;
}

static int audio_dev_ctrl_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	mutex_lock(&audio_dev_ctrl.lock);
	if (audio_dev_ctrl.opened) {
		pr_err("audio: busy\n");
		rc = -EBUSY;
		goto error;
	} else
		audio_dev_ctrl.opened = 1;
	file->private_data = &audio_dev_ctrl;
error:
	mutex_unlock(&audio_dev_ctrl.lock);
	return rc;
}

static int audio_dev_ctrl_release(struct inode *inode, struct file *file)
{

	mutex_lock(&audio_dev_ctrl.lock);
	audio_dev_ctrl.opened = 0;
	mutex_unlock(&audio_dev_ctrl.lock);
	return 0;
}

static const struct file_operations audio_dev_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = audio_dev_ctrl_open,
	.release = audio_dev_ctrl_release,
	.ioctl = audio_dev_ctrl_ioctl,
};


struct miscdevice audio_dev_ctrl_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_dev_ctrl",
	.fops	= &audio_dev_ctrl_fops,
};

static int __init audio_dev_ctrl_init(void)
{
	mutex_init(&audio_dev_ctrl.lock);
	audio_dev_ctrl.opened = 0;
	audio_dev_ctrl.num_dev = 0;
	audio_dev_ctrl.voice_rx_dev = NULL;
	audio_dev_ctrl.voice_tx_dev = NULL;
	return misc_register(&audio_dev_ctrl_misc);
}

static void __exit audio_dev_ctrl_exit(void)
{
}
module_init(audio_dev_ctrl_init);
module_exit(audio_dev_ctrl_exit);

MODULE_DESCRIPTION("MSM 7K Audio Device Control driver");
MODULE_LICENSE("Dual BSD/GPL");
