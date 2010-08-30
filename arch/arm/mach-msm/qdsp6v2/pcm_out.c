/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#define BUFSZ (4800)

struct pcm {
	struct mutex lock;
	struct audio_client *ac;
	uint32_t sample_rate;
	uint32_t channel_count;
	size_t buffer_size;
	int run;
};

static long pcm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pcm *pcm = file->private_data;
	int rc = 0;

	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		memset(&stats, 0, sizeof(stats));
		if (copy_to_user((void *) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}

	mutex_lock(&pcm->lock);
	switch (cmd) {
	case AUDIO_SET_VOLUME: {
		int vol;
		if (copy_from_user(&vol, (void *) arg, sizeof(vol))) {
			rc = -EFAULT;
			break;
		}
		break;
	}
	case AUDIO_START: {
		struct audio_buffer *ab;
		struct audio_client *ac;

		if (!pcm->run) {
			if (q6asm_out_run(pcm->ac)) {
				rc = -ENOMEM;
				break;
			}
			ac = pcm->ac;
			ab = &ac->buf[0];
			q6asm_write(ac, ab);
			ab = &ac->buf[1];
			q6asm_write(ac, ab);
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
		}
		break;
	case AUDIO_STOP:
		break;
	case AUDIO_FLUSH:
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		pr_info("AUDIO_SET_CONFIG\n");
		if (copy_from_user(&config, (void *) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		if (config.channel_count < 1 || config.channel_count > 2) {
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
		pr_info("AUDIO_GET_CONFIG\n");
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
	case AUDIO_SET_EQ: {
		struct msm_audio_eq_stream_config eq_config;
		if (copy_from_user(&eq_config, (void *) arg,
						sizeof(eq_config))) {
			rc = -EFAULT;
			break;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&pcm->lock);
	return rc;
}


static int pcm_open(struct inode *inode, struct file *file)
{
	struct pcm *pcm;

	pr_info("[%s:%s] open\n", __MM_FILE__, __func__);
	pcm = kzalloc(sizeof(struct pcm), GFP_KERNEL);

	if (!pcm)
		return -ENOMEM;

	pcm->channel_count = 2;
	pcm->sample_rate = 44100;
	pcm->buffer_size = BUFSZ;
	pcm->ac = q6asm_out_open(pcm->buffer_size,
				pcm->sample_rate,
				pcm->channel_count);
	if (!pcm->ac) {
		pr_info("%s: q6asm_out_open failed\n", __func__);
		kfree(pcm);
		return -ENOMEM;
	}

	mutex_init(&pcm->lock);
	file->private_data = pcm;
	return 0;
}

static ssize_t pcm_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct pcm *pcm = file->private_data;
	struct audio_client *ac;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer;

	pr_info("%s\n", __func__);

	ac = pcm->ac;
	if (!ac)
		return -ENODEV;

	while (count > 0) {
		ab = ac->buf + ac->cpu_buf;

		if (ab->used) {
			if (!wait_event_timeout(ac->wait, (ab->used == 0),
								5*HZ)) {
				pr_err("[%s:%s] timeout. dsp dead?\n",
						__MM_FILE__, __func__);
			}
		}

		xfer = count;
		if (xfer > ab->size)
			xfer = ab->size;

		if (copy_from_user(ab->data, buf, xfer))
			return -EFAULT;

		buf += xfer;
		count -= xfer;

		ab->used = 1;
		ab->actual_size = xfer;

		/* TODO: change to use pause/resume/run */
		if (pcm->run)
			q6asm_write(ac, ab);
		ac->cpu_buf ^= 1;
	}

	return buf - start;
}

static int pcm_release(struct inode *inode, struct file *file)
{
	struct pcm *pcm = file->private_data;
	if (pcm->ac)
		q6asm_close(pcm->ac);
	kfree(pcm);
	pr_info("[%s:%s] release\n", __MM_FILE__, __func__);
	return 0;
}

static const struct file_operations pcm_fops = {
	.owner		= THIS_MODULE,
	.open		= pcm_open,
	.write		= pcm_write,
	.release	= pcm_release,
	.unlocked_ioctl	= pcm_ioctl,
};

struct miscdevice pcm_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_out",
	.fops	= &pcm_fops,
};

static int __init pcm_init(void)
{
	return misc_register(&pcm_misc);
}

device_initcall(pcm_init);
