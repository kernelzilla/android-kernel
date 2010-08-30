/* linux/sound/soc/msm/qsd8k-pcm.c
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>

#include "qsd-pcm.h"

struct snd_pcm_runtime *runtime_dummy;
static int rc = 1;

#define SND_DRIVER        "snd_qsd"
#define MAX_PCM_DEVICES	SNDRV_CARDS
#define MAX_PCM_SUBSTREAMS 1

struct snd_qsd {
	struct snd_card *card;
	struct snd_pcm *pcm;
};

struct audio_locks the_locks;

static unsigned convert_dsp_samp_index(unsigned index)
{
	switch (index) {
	case 48000:
		return 3;
	case 44100:
		return 4;
	case 32000:
		return 5;
	case 24000:
		return 6;
	case 22050:
		return 7;
	case 16000:
		return 8;
	case 12000:
		return 9;
	case 11025:
		return 10;
	case 8000:
		return 11;
	default:
		return 3;
	}
}

static struct snd_pcm_hardware qsd_pcm_playback_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED,
	.formats = USE_FORMATS,
	.rates = USE_RATE,
	.rate_min = USE_RATE_MIN,
	.rate_max = USE_RATE_MAX,
	.channels_min = USE_CHANNELS_MIN,
	.channels_max = USE_CHANNELS_MAX,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min = USE_PERIODS_MIN,
	.periods_max = USE_PERIODS_MAX,
	.fifo_size = 0,
};

static struct snd_pcm_hardware qsd_pcm_capture_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED,
	.formats = USE_FORMATS,
	.rates = USE_RATE,
	.rate_min = USE_RATE_MIN,
	.rate_max = USE_RATE_MAX,
	.channels_min = USE_CHANNELS_MIN,
	.channels_max = USE_CHANNELS_MAX,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = MAX_PERIOD_SIZE,
	.periods_min = USE_PERIODS_MIN,
	.periods_max = USE_PERIODS_MAX,
	.fifo_size = 0,
};

/* Conventional and unconventional sample rate supported */ static
unsigned int supported_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static int qsd_pcm_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;

	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_pcm_format_struct_type cad_write_pcm_fmt;
	u32 stream_device[1];

	if (prtd->enabled)
		return 0;

	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	prtd->pcm_buf_pos = 0;

	cad_stream_info.app_type = CAD_STREAM_APP_PLAYBACK;
	cad_stream_info.priority = 0;
	cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
	cad_stream_info.ses_buf_max_size = prtd->pcm_count;

	mutex_lock(&the_locks.lock);
	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
		       &cad_stream_info,
		       sizeof(struct cad_stream_info_struct_type));
	mutex_unlock(&the_locks.lock);
	if (rc)
		printk(KERN_ERR "cad ioctl failed\n");

	stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_RX ;
	cad_stream_dev.device = (u32 *) &stream_device[0];
	cad_stream_dev.device_len = 1;
	mutex_lock(&the_locks.lock);

	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
		       &cad_stream_dev,
		       sizeof(struct cad_stream_device_struct_type));
	mutex_unlock(&the_locks.lock);
	if (rc)
		printk(KERN_ERR "cad ioctl  failed\n");

	cad_write_pcm_fmt.us_ver_id = CAD_WRITE_PCM_VERSION_10;
	cad_write_pcm_fmt.pcm.us_sample_rate =
	    convert_dsp_samp_index(runtime->rate);
	cad_write_pcm_fmt.pcm.us_channel_config = runtime->channels;
	cad_write_pcm_fmt.pcm.us_width = 1;
	cad_write_pcm_fmt.pcm.us_sign = 0;

	mutex_lock(&the_locks.lock);
	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_CONFIG,
		       &cad_write_pcm_fmt,
		       sizeof(struct cad_write_pcm_format_struct_type));
	mutex_unlock(&the_locks.lock);
	if (rc)
		printk(KERN_ERR "cad ioctl failed\n");

	mutex_lock(&the_locks.lock);
	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
		NULL, 0);
	mutex_unlock(&the_locks.lock);
	if (rc)
		printk(KERN_ERR "cad ioctl failed\n");
	else
		prtd->enabled = 1;

	return rc;
}

static int qsd_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
qsd_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;

	if (prtd->pcm_irq_pos == prtd->pcm_size)
		prtd->pcm_irq_pos = 0;
	return bytes_to_frames(runtime, (prtd->pcm_irq_pos));
}

void alsa_event_cb_playback(void)
{
	if (runtime_dummy) {
		struct qsd_audio *prtd = runtime_dummy->private_data;
		prtd->pcm_irq_pos += prtd->pcm_count;
		snd_pcm_period_elapsed(prtd->playback_substream);
	}
}
void alsa_event_cb_capture(u32 event, void *evt_packet,
			u32 evt_packet_len, void *client_data)
{
	struct qsd_audio *prtd = client_data;
	prtd->pcm_irq_pos += prtd->pcm_count;
	snd_pcm_period_elapsed(prtd->capture_substream);
}


static int qsd_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd;
	struct cad_event_struct_type alsa_event;
	int ret = 0;

	runtime_dummy = runtime;

	prtd = kzalloc(sizeof(struct qsd_audio), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		return ret;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		printk(KERN_INFO "Stream = SNDRV_PCM_STREAM_PLAYBACK\n");
		runtime->hw = qsd_pcm_playback_hardware;
		prtd->dir = SNDRV_PCM_STREAM_PLAYBACK;
		prtd->playback_substream = substream;
		prtd->cos.op_code = CAD_OPEN_OP_WRITE;
	} else {
		printk(KERN_INFO "Stream = SNDRV_PCM_STREAM_CAPTURE\n");
		runtime->hw = qsd_pcm_capture_hardware;
		prtd->dir = SNDRV_PCM_STREAM_CAPTURE;
		prtd->capture_substream = substream;
		prtd->cos.op_code = CAD_OPEN_OP_READ;
	}

	ret = snd_pcm_hw_constraint_list(runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);
	if (ret < 0) {
		kfree(prtd);
		return ret;
	}

	runtime->private_data = prtd;

	prtd->cos.format = CAD_FORMAT_PCM;

	mutex_lock(&the_locks.lock);
	prtd->cad_w_handle = cad_open(&prtd->cos);
	mutex_unlock(&the_locks.lock);

	mutex_lock(&the_locks.lock);
	if  (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		alsa_event.callback = &alsa_event_cb_capture;
		alsa_event.client_data = prtd;

		ret = cad_ioctl(prtd->cad_w_handle,
			CAD_IOCTL_CMD_SET_STREAM_EVENT_LSTR,
			&alsa_event, sizeof(struct cad_event_struct_type));
		if (ret) {
			mutex_unlock(&the_locks.lock);
			cad_close(prtd->cad_w_handle);
			kfree(prtd);
			return ret;
		}
	} else
		register_cb(&alsa_event_cb_playback);
	mutex_unlock(&the_locks.lock);
	prtd->enabled = 0;

	return 0;
}

static int qsd_pcm_playback_copy(struct snd_pcm_substream *substream, int a,
				 snd_pcm_uframes_t hwoff, void __user *buf,
				 snd_pcm_uframes_t frames)
{
	int fbytes = 0;
	size_t xfer;

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;

	fbytes = frames_to_bytes(runtime, frames);
	prtd->cbs.buffer = (void *)buf;
	prtd->cbs.phys_addr = 0;
	prtd->cbs.max_size = fbytes;
	prtd->cbs.actual_size = fbytes;

	prtd->pcm_buf_pos += fbytes;
	mutex_lock(&the_locks.lock);
	xfer = cad_write(prtd->cad_w_handle, &prtd->cbs);
	mutex_unlock(&the_locks.lock);

	return 0;
}

static int qsd_pcm_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;
	int ret;

	mutex_lock(&the_locks.lock);
	ret = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_STREAM_PAUSE,
			NULL, 0);
	cad_close(prtd->cad_w_handle);
	mutex_unlock(&the_locks.lock);
	prtd->enabled = 0;

	/*
	 * TODO: Deregister the async callback handler.
	 * Currently cad provides no interface to do so.
	 */
	register_cb(NULL);
	kfree(prtd);

	return 0;
}

static int qsd_pcm_capture_copy(struct snd_pcm_substream *substream, int a,
				 snd_pcm_uframes_t hwoff, void __user *buf,
				 snd_pcm_uframes_t frames)
{
	int fbytes = 0;
	size_t xfer;

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;

	fbytes = frames_to_bytes(runtime, frames);
	fbytes = fbytes;

	prtd->cbs.buffer = (void *)buf;
	prtd->cbs.phys_addr = 0;
	prtd->cbs.max_size = fbytes;
	prtd->cbs.actual_size = fbytes;

	mutex_lock(&the_locks.lock);
	xfer = cad_read(prtd->cad_w_handle, &prtd->cbs);
	mutex_unlock(&the_locks.lock);

	prtd->pcm_buf_pos += fbytes;

	if (xfer < fbytes)
		return -EIO;

	return 0;
}

static snd_pcm_uframes_t
qsd_pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;

	return bytes_to_frames(runtime, (prtd->pcm_irq_pos));
}

static int qsd_pcm_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;
	int ret;

	mutex_lock(&the_locks.lock);
	ret = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_STREAM_PAUSE,
			NULL, 0);
	cad_close(prtd->cad_w_handle);
	mutex_unlock(&the_locks.lock);

	/*
	 * TODO: Deregister the async callback handler.
	 * Currently cad provides no interface to do so.
	 */
	kfree(prtd);

	return 0;
}

static int qsd_pcm_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;
	int rc = 0;

	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_pcm_format_struct_type cad_write_pcm_fmt;
	u32 stream_device[1];

	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	prtd->pcm_buf_pos = 0;

	cad_stream_info.app_type = CAD_STREAM_APP_RECORD;
	cad_stream_info.priority = 0;
	cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
	cad_stream_info.ses_buf_max_size = prtd->pcm_count;

	if (prtd->enabled)
		return 0;

	mutex_lock(&the_locks.lock);
	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
		&cad_stream_info,
		sizeof(struct cad_stream_info_struct_type));
	if (rc) {
		mutex_unlock(&the_locks.lock);
		return rc;
	}

	stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_TX ;
	cad_stream_dev.device = (u32 *) &stream_device[0];
	cad_stream_dev.device_len = 1;

	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
	       &cad_stream_dev,
	       sizeof(struct cad_stream_device_struct_type));
	if (rc) {
		mutex_unlock(&the_locks.lock);
		return rc;
	}

	cad_write_pcm_fmt.us_ver_id = CAD_WRITE_PCM_VERSION_10;
	cad_write_pcm_fmt.pcm.us_sample_rate =
	    convert_dsp_samp_index(runtime->rate);
	cad_write_pcm_fmt.pcm.us_channel_config = runtime->channels;
	cad_write_pcm_fmt.pcm.us_width = 1;
	cad_write_pcm_fmt.pcm.us_sign = 0;

	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_CONFIG,
	       &cad_write_pcm_fmt,
	       sizeof(struct cad_write_pcm_format_struct_type));
	if (rc) {
		mutex_unlock(&the_locks.lock);
		return rc;
	}
	rc = cad_ioctl(prtd->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
			NULL, 0);
	mutex_unlock(&the_locks.lock);
	if (!rc)
		prtd->enabled = 1;
	return rc;
}


static int qsd_pcm_copy(struct snd_pcm_substream *substream, int a,
			snd_pcm_uframes_t hwoff, void __user *buf,
			snd_pcm_uframes_t frames)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = qsd_pcm_playback_copy(substream, a, hwoff, buf, frames);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = qsd_pcm_capture_copy(substream, a, hwoff, buf, frames);
	return ret;
}

static int qsd_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = qsd_pcm_playback_close(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = qsd_pcm_capture_close(substream);
	return ret;
}
static int qsd_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = qsd_pcm_playback_prepare(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = qsd_pcm_capture_prepare(substream);
	return ret;
}

static snd_pcm_uframes_t qsd_pcm_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = qsd_pcm_playback_pointer(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = qsd_pcm_capture_pointer(substream);
	return ret;
}

int qsd_pcm_hw_params(struct snd_pcm_substream *substream,
		      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->pcm->device & 1) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
		runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	}
	return 0;
}

struct snd_pcm_ops qsd_pcm_ops = {
	.open = qsd_pcm_open,
	.copy = qsd_pcm_copy,
	.hw_params = qsd_pcm_hw_params,
	.close = qsd_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.prepare = qsd_pcm_prepare,
	.trigger = qsd_pcm_trigger,
	.pointer = qsd_pcm_pointer,
};
EXPORT_SYMBOL_GPL(qsd_pcm_ops);

MODULE_DESCRIPTION("PCM module platform driver for 8k");
MODULE_LICENSE("GPL v2");
