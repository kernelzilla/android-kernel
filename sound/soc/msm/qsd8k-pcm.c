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
#include <asm/dma.h>
#include <linux/dma-mapping.h>

#include "qsd-pcm.h"

static int rc = 1;

#define SND_DRIVER        "snd_qsd"
#define MAX_PCM_DEVICES	SNDRV_CARDS
#define MAX_PCM_SUBSTREAMS 1

struct snd_qsd {
	struct snd_card *card;
	struct snd_pcm *pcm;
};

struct qsd_ctl qsd_glb_ctl;
EXPORT_SYMBOL(qsd_glb_ctl);
struct audio_locks the_locks;
EXPORT_SYMBOL(the_locks);

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

int qsd_audio_volume_update(struct qsd_audio *prtd)
{

	int rc = 0;
	struct cad_flt_cfg_strm_vol cad_strm_volume;
	struct cad_filter_struct flt;

	printk(KERN_INFO "qsd_audio_volume_update: updating volume");
	memset(&cad_strm_volume, 0, sizeof(struct cad_flt_cfg_strm_vol));
	memset(&flt, 0, sizeof(struct cad_filter_struct));

	cad_strm_volume.volume = qsd_glb_ctl.strm_volume;
	flt.filter_type = CAD_DEVICE_FILTER_TYPE_VOL;
	flt.format_block = &cad_strm_volume;
	flt.cmd = CAD_FILTER_CONFIG_STREAM_VOLUME;
	flt.format_block_len = sizeof(struct cad_flt_cfg_strm_vol);

	rc = cad_ioctl(prtd->cad_w_handle,
		CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG,
		&flt,
		sizeof(struct cad_filter_struct));
	if (rc)
		printk(KERN_ERR "cad_ioctl() set volume failed\n");
	return rc;
}

static int qsd_pcm_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;

	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_pcm_format_struct_type cad_write_pcm_fmt;
	u32 stream_device[1];

	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	prtd->pcm_buf_pos = 0;

	if (prtd->enabled)
		return 0;

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

void alsa_event_cb_playback(u32 event, void *evt_packet,
			u32 evt_packet_len, void *client_data)
{
	struct qsd_audio *prtd = client_data;
	if (event == CAD_EVT_STATUS_EOS) {

		prtd->eos_ack = 1;
		pr_info("EOS Received\n");
		wake_up(&the_locks.eos_wait);
		return ;
	}

	prtd->pcm_irq_pos += prtd->pcm_count;
	snd_pcm_period_elapsed(prtd->playback_substream);
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

	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
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
	if  (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		alsa_event.callback = &alsa_event_cb_capture;
	else
		alsa_event.callback = &alsa_event_cb_playback;

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
	int rc;

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

	mutex_lock(&the_locks.mixer_lock);
	if (qsd_glb_ctl.update) {
		rc = qsd_audio_volume_update(prtd);
		qsd_glb_ctl.update = 0;
	}
	mutex_unlock(&the_locks.mixer_lock);

	return 0;
}

static int qsd_pcm_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct qsd_audio *prtd = runtime->private_data;
	int ret = 0;

	if (prtd->enabled) {
		mutex_lock(&the_locks.lock);
		cad_ioctl(prtd->cad_w_handle,
			CAD_IOCTL_CMD_STREAM_END_OF_STREAM,
			NULL, 0);
		mutex_unlock(&the_locks.lock);

		ret = wait_event_interruptible(the_locks.eos_wait,
					prtd->eos_ack);

		if (!prtd->eos_ack)
			pr_err("EOS Failed\n");

	}

	prtd->eos_ack = 0;
	mutex_lock(&the_locks.lock);
	cad_close(prtd->cad_w_handle);
	mutex_unlock(&the_locks.lock);
	prtd->enabled = 0;

	/*
	 * TODO: Deregister the async callback handler.
	 * Currently cad provides no interface to do so.
	 */
	kfree(prtd);

	return ret;
}

static int qsd_pcm_capture_copy(struct snd_pcm_substream *substream, int a,
				 snd_pcm_uframes_t hwoff, void __user *buf,
				 snd_pcm_uframes_t frames)
{
	int fbytes = 0;
	size_t xfer;
	int rc = 0;

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

	return rc;
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

	mutex_lock(&the_locks.lock);
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

static int qsd_pcm_remove(struct platform_device *devptr)
{
	struct snd_soc_device *socdev = platform_get_drvdata(devptr);
	snd_soc_free_pcms(socdev);
	kfree(socdev->codec);
	platform_set_drvdata(devptr, NULL);
	return 0;
}

static int qsd_pcm_new(struct snd_card *card,
			struct snd_soc_dai *codec_dai,
			struct snd_pcm *pcm)
{
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	return 0;
}

struct snd_soc_platform qsd_soc_platform = {
	.name		= "qsd-audio",
	.remove         = qsd_pcm_remove,
	.pcm_ops 	= &qsd_pcm_ops,
	.pcm_new	= qsd_pcm_new,
};
EXPORT_SYMBOL(qsd_soc_platform);

static int __init qsd_soc_platform_init(void)
{
	return snd_soc_register_platform(&qsd_soc_platform);
}
module_init(qsd_soc_platform_init);

static void __exit qsd_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&qsd_soc_platform);
}
module_exit(qsd_soc_platform_exit);

MODULE_DESCRIPTION("PCM module platform driver");
MODULE_LICENSE("GPL v2");
