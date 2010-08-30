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

#include <mach/debug_mm.h>

#include "q6asm.h"

struct pcm {
	struct mutex lock;
	struct audio_client *ac;
	uint32_t sample_rate;
	uint32_t channel_count;
	uint32_t buffer_size;
	uint32_t rec_mode;
	int run;
};

#define BUFSZ (4096)

static long pcm_in_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pcm *pcm = file->private_data;
	int rc = 0;

	mutex_lock(&pcm->lock);

	switch (cmd) {
	case AUDIO_SET_VOLUME:
		break;
	case AUDIO_GET_STATS: {
		struct msm_audio_stats stats;
		memset(&stats, 0, sizeof(stats));
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			rc = -EFAULT;
		break;
	}
	case AUDIO_START: {
		struct audio_buffer *ab;
		struct audio_client *ac;
		if (pcm->ac) {
			rc = -EBUSY;
		} else {
			pcm->ac = q6asm_in_open(pcm->buffer_size,
					pcm->sample_rate, pcm->channel_count);
			if (!pcm->ac)
				rc = -ENOMEM;

			ac = pcm->ac;
			if (!pcm->run) {
				if (q6asm_in_run(ac)) {
					pr_info("%s: RUn command failed\n",
								__func__);
					rc = -ENOMEM;
				}
			}

			ac->dsp_buf = 0;
			ab = &ac->buf[0];
			q6asm_read(ac, ab);
			ab->used = 1;

			ac->dsp_buf = 1;
			ab = &ac->buf[1];
			q6asm_read(ac, ab);
			ab->used = 1;

			pcm->run = 1;
		}
		break;
	}
	case AUDIO_GET_SESSION_ID: {
		unsigned short session = 1;
		pr_info("AUDIO_GET_SESSION_ID\n");
		if (copy_to_user((void *) arg, &session,
					sizeof(unsigned short)))
			rc = -EFAULT;
		break;
		}
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (copy_from_user(&config, (void *) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		if (!config.channel_count || config.channel_count > 2) {
			rc = -EINVAL;
			break;
		}
		if (config.sample_rate < 8000 || config.sample_rate > 48000) {
			rc = -EINVAL;
			break;
		}
		if (config.buffer_size < 128 || config.buffer_size > 8192) {
			rc = -EINVAL;
			break;
		}

		pcm->sample_rate = config.sample_rate;
		pcm->channel_count = config.channel_count;
		pcm->buffer_size = config.buffer_size;
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = pcm->buffer_size;
		config.buffer_count = 2;
		config.sample_rate = pcm->sample_rate;
		config.channel_count = pcm->channel_count;
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		if (copy_to_user((void *) arg, &config, sizeof(config)))
			rc = -EFAULT;
		break;
	}
	default:
		rc = -EINVAL;
	}

	mutex_unlock(&pcm->lock);
	return rc;
}

static int pcm_in_open(struct inode *inode, struct file *file)
{
	struct pcm *pcm;

	pr_info("[%s:%s] open\n", __MM_FILE__, __func__);
	pcm = kzalloc(sizeof(struct pcm), GFP_KERNEL);

	if (!pcm)
		return -ENOMEM;

	pcm->channel_count = 1;
	pcm->sample_rate = 8000;
	pcm->buffer_size = BUFSZ;
	mutex_init(&pcm->lock);
	file->private_data = pcm;
	return 0;
}

static ssize_t pcm_in_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos)
{
	struct pcm *pcm = file->private_data;
	struct audio_client *ac;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer;
	int res;

	ac = pcm->ac;
	if (!ac) {
		res = -ENODEV;
		goto fail;
	}
	while (count > 0) {
		ab = ac->buf + ac->cpu_buf;
		if (ab->used) {
			if (!wait_event_timeout(ac->wait, (ab->used == 0),
								5*HZ)) {
				pr_err("[%s:%s] timeout. dsp dead?\n",
						__MM_FILE__, __func__);
			}
		}
		xfer = ab->actual_size;
		if (copy_to_user(buf, ab->data, xfer)) {
			res = -EFAULT;
			goto fail;
		}
		buf += xfer;
		count -= xfer;

		ab->used = 1;
		ac->dsp_buf = ac->cpu_buf;
		if (pcm->run)
			q6asm_read(ac, ab);
		ac->cpu_buf ^= 1;
	}
fail:
	res = buf - start;
	return res;
}

static int pcm_in_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct pcm *pcm = file->private_data;
	if (pcm->ac)
		rc = q6asm_close(pcm->ac);
	kfree(pcm);
	pr_info("[%s:%s] release\n", __MM_FILE__, __func__);
	return rc;
}

static const struct file_operations pcm_in_fops = {
	.owner		= THIS_MODULE,
	.open		= pcm_in_open,
	.read		= pcm_in_read,
	.release	= pcm_in_release,
	.unlocked_ioctl	= pcm_in_ioctl,
};

struct miscdevice pcm_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_in",
	.fops	= &pcm_in_fops,
};

static int __init pcm_in_init(void)
{
	return misc_register(&pcm_in_misc);
}

device_initcall(pcm_in_init);
