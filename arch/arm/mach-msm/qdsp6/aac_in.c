/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/msm_audio.h>
#include <linux/msm_audio_aac.h>
#include <mach/msm_qdsp6_audio.h>
#include <mach/debug_audio_mm.h>

struct aac {
	struct mutex lock;
	struct msm_audio_aac_enc_config cfg;
	struct msm_audio_stream_config str_cfg;
	struct audio_client *audio_client;
	struct msm_voicerec_mode voicerec_mode;
};

static long q6_aac_in_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{
	struct aac *aac = file->private_data;
	int rc = 0;

	mutex_lock(&aac->lock);
	switch (cmd) {
	case AUDIO_SET_VOLUME:
		break;
	case AUDIO_GET_STATS:
	{
		struct msm_audio_stats stats;
		memset(&stats, 0, sizeof(stats));
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}
	case AUDIO_START:
	{
		uint32_t acdb_id;
		if (arg == 0) {
			acdb_id = 0;
		} else {
			if (copy_from_user(&acdb_id, (void *) arg,
					sizeof(acdb_id))) {
				rc = -EFAULT;
				break;
			}
		}
		if (aac->audio_client) {
			rc = -EBUSY;
			break;
		} else {
			aac->audio_client = q6audio_open_aac(
					aac->str_cfg.buffer_size,
					aac->cfg.sample_rate,
					aac->cfg.channels,
					aac->cfg.bit_rate,
					aac->cfg.stream_format,
					aac->voicerec_mode.rec_mode, acdb_id);

			if (aac->audio_client < 0) {
				rc = -ENOMEM;
				break;
			}
		}
		break;
	}
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_SET_INCALL: {
		if (copy_from_user(&aac->voicerec_mode,
			(void *)arg, sizeof(struct msm_voicerec_mode)))
			rc = -EFAULT;

		if (aac->voicerec_mode.rec_mode != AUDIO_FLAG_READ
			&& aac->voicerec_mode.rec_mode !=
			AUDIO_FLAG_INCALL_MIXED) {
			aac->voicerec_mode.rec_mode = AUDIO_FLAG_READ;
			MM_ERR("Invalid rec_mode\n");
			rc = -EINVAL;
		}
		break;
	}
	case AUDIO_GET_STREAM_CONFIG:
		if (copy_to_user((void *)arg, &aac->str_cfg,
			sizeof(struct msm_audio_stream_config)))
			rc = -EFAULT;
		break;
	case AUDIO_SET_STREAM_CONFIG:
		if (copy_from_user(&aac->str_cfg, (void *)arg,
			sizeof(struct msm_audio_stream_config))) {
			rc = -EFAULT;
			break;
		}
		if (aac->str_cfg.buffer_size < 519) {
			MM_ERR("Buffer size too small\n");
			rc = -EINVAL;
			break;
		}
		if (aac->str_cfg.buffer_count != 2)
			MM_INFO("Buffer count set to 2\n");

		break;
	case AUDIO_SET_AAC_ENC_CONFIG:
		if (copy_from_user(&aac->cfg, (void *) arg,
				 sizeof(struct msm_audio_aac_enc_config))) {
			rc = -EFAULT;
		}
		if (aac->cfg.channels != 1) {
			MM_ERR("only mono is supported\n");
			rc = -EINVAL;
		}
		if (aac->cfg.sample_rate != 48000) {
			MM_ERR("only 48KHz is supported\n");
			rc = -EINVAL;
		}
		if (aac->cfg.stream_format != AUDIO_AAC_FORMAT_RAW &&
			aac->cfg.stream_format != AUDIO_AAC_FORMAT_ADTS) {
			MM_ERR("unsupported AAC format\n");
			rc = -EINVAL;
		}
		break;
	case AUDIO_GET_AAC_ENC_CONFIG:
		if (copy_to_user((void *) arg, &aac->cfg,
				 sizeof(struct msm_audio_aac_enc_config))) {
			rc = -EFAULT;
		}
		break;
	default:
		rc = -EINVAL;
	}

	mutex_unlock(&aac->lock);
	return rc;
}

static int q6_aac_in_open(struct inode *inode, struct file *file)
{

	struct aac *aac;
	aac = kmalloc(sizeof(struct aac), GFP_KERNEL);
	if (aac == NULL) {
		MM_ERR("Could not allocate memory for aac driver\n");
		return -ENOMEM;
	}

	mutex_init(&aac->lock);
	file->private_data = aac;
	aac->audio_client = NULL;
	aac->str_cfg.buffer_size = 519;
	aac->str_cfg.buffer_count = 2;
	aac->cfg.channels = 1;
	aac->cfg.bit_rate = 192000;
	aac->cfg.stream_format = AUDIO_AAC_FORMAT_ADTS;
	aac->cfg.sample_rate = 48000;
	aac->voicerec_mode.rec_mode = AUDIO_FLAG_READ;

	return 0;
}

static ssize_t q6_aac_in_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos)
{
	struct audio_client *ac;
	struct audio_buffer *ab;
	const char __user *start = buf;
	struct aac *aac = file->private_data;
	int xfer = 0;
	int res;

	mutex_lock(&aac->lock);
	ac = aac->audio_client;
	if (!ac) {
		res = -ENODEV;
		goto fail;
	}
	while (count > xfer) {
		ab = ac->buf + ac->cpu_buf;

		if (ab->used)
			wait_event(ac->wait, (ab->used == 0));

		xfer = ab->actual_size;

		if (copy_to_user(buf, ab->data, xfer)) {
			res = -EFAULT;
			goto fail;
		}

		buf += xfer;
		count -= xfer;

		ab->used = 1;
		q6audio_read(ac, ab);
		ac->cpu_buf ^= 1;
	}
	res = buf - start;
fail:
	mutex_unlock(&aac->lock);

	return res;
}

static int q6_aac_in_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct aac *aac = file->private_data;

	mutex_lock(&aac->lock);
	if (aac->audio_client)
		rc = q6audio_close(aac->audio_client);
	mutex_unlock(&aac->lock);
	kfree(aac);
	return rc;
}

static const struct file_operations q6_aac_in_fops = {
	.owner		= THIS_MODULE,
	.open		= q6_aac_in_open,
	.read		= q6_aac_in_read,
	.release	= q6_aac_in_release,
	.unlocked_ioctl	= q6_aac_in_ioctl,
};

struct miscdevice q6_aac_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_aac_in",
	.fops	= &q6_aac_in_fops,
};

static int __init q6_aac_in_init(void)
{
	return misc_register(&q6_aac_in_misc);
}

device_initcall(q6_aac_in_init);
