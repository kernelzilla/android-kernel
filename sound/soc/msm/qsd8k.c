/* linux/sound/soc/msm/qsd8k.c
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

static struct platform_device *qsd_audio_snd_device;
struct qsd_ctl qsd_glb_ctl;

struct snd_soc_dai qsd8k_dai = {
	.name = "ASOC",
	.playback = {
		.stream_name 	= "Playback",
		.channels_min 	= USE_CHANNELS_MIN,
		.channels_max 	= USE_CHANNELS_MAX,
		.rates 		= USE_RATE,
		.rate_min 	= USE_RATE_MIN,
		.rate_max 	= USE_RATE_MAX,
		.formats 	= USE_FORMATS,
	},
	.capture = {
		.stream_name 	= "Capture",
		.channels_min 	= USE_CHANNELS_MIN,
		.channels_max 	= USE_CHANNELS_MAX,
		.rate_min 	= USE_RATE_MIN,
		.rates 		= USE_RATE,
		.formats 	= USE_FORMATS,
	},
};

struct snd_soc_dai qsd_cpudai = {
	.name = "QSD_8650",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.playback = {
		.channels_min 	= USE_CHANNELS_MIN,
		.channels_max 	= USE_CHANNELS_MAX,
		.rates 		= USE_RATE,
		.rate_min 	= USE_RATE_MIN,
		.rate_max 	= USE_RATE_MAX,
		.formats 	= USE_FORMATS,
	},
	.capture = {
		.channels_min 	= USE_CHANNELS_MIN,
		.channels_max 	= USE_CHANNELS_MAX,
		.rate_min	= USE_RATE_MIN,
		.rates 		= USE_RATE,
		.formats 	= USE_FORMATS,
	},
};


static struct snd_soc_dai_link qsd_dai = {
	.name 		= "ASOC",
	.stream_name 	= "ASOC",
	.codec_dai 	= &qsd8k_dai,
	.cpu_dai 	= &qsd_cpudai,
};


struct snd_soc_machine snd_soc_machine_qsd = {
	.name 		= "qsd-audio",
	.dai_link	= &qsd_dai,
	.num_links 	= 1,
};

static int snd_qsd_route_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1; /* Device */
	uinfo->value.integer.min = CAD_HW_DEVICE_ID_HANDSET_MIC;
	uinfo->value.integer.max = CAD_HW_DEVICE_ID_DEFAULT_RX;
	return 0;
}

static int snd_qsd_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
			(uint32_t) qsd_glb_ctl.playback_device;
	ucontrol->value.integer.value[1] =
			(uint32_t) qsd_glb_ctl.capture_device;
	return 0;
}

static int snd_get_device_type(int device)
{
	switch (device) {
	case CAD_HW_DEVICE_ID_HANDSET_MIC:
	case CAD_HW_DEVICE_ID_HEADSET_MIC:
	case CAD_HW_DEVICE_ID_BT_SCO_MIC:
	case CAD_HW_DEVICE_ID_DEFAULT_TX:
		return CAD_TX_DEVICE;
	case CAD_HW_DEVICE_ID_HANDSET_SPKR:
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO:
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO:
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MIC:
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MONO:
	case CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO:
	case CAD_HW_DEVICE_ID_BT_SCO_SPKR:
	case CAD_HW_DEVICE_ID_BT_A2DP_SPKR:
	case CAD_HW_DEVICE_ID_DEFAULT_RX:
		return CAD_RX_DEVICE;
	default:
		return -ENODEV;
	}
}

static int snd_qsd_route_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	int device, direction;

	device = ucontrol->value.integer.value[0];
	direction = snd_get_device_type(device);

	if (direction < 0)
		return direction;

	rc = audio_switch_device(device);
	if (rc < 0) {
		printk(KERN_ERR "audio_switch_device  failed\n");
		return rc;
	}

	if (CAD_RX_DEVICE == direction)
		qsd_glb_ctl.playback_device = device;
	else /* CAD_TX_DEVICE */
		qsd_glb_ctl.capture_device = device;

	return 0;
}

static int snd_vol_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1; /* Volume */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;
	return 0;
}

static int snd_vol_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.volume;
	return 0;
}

static int snd_vol_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;

	rc = audio_set_device_volume(ucontrol->value.integer.value[0]);
	if (rc)
		printk(KERN_ERR "audio_set_device_volume failed\n");
	else
		qsd_glb_ctl.volume = ucontrol->value.integer.value[0];

	return rc;
}

static int snd_mute_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1; /* MUTE */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int snd_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.mute;
	return 0;
}

static int snd_mute_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;

	rc = audio_set_device_mute(ucontrol->value.integer.value[0]);
	if (rc)
		printk(KERN_ERR "audio_set_device_mute failed\n");
	else
		qsd_glb_ctl.mute = ucontrol->value.integer.value[0];
	return rc;
}

#define QSD_EXT(xname, xindex, fp_info, fp_get, fp_put, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = fp_info,\
  .get = fp_get, .put = fp_put, \
  .private_value = addr, \
}

static struct snd_kcontrol_new snd_qsd_controls[] = {
	QSD_EXT("Master Route", 1, snd_qsd_route_info, \
			 snd_qsd_route_get, snd_qsd_route_put, 0),
	QSD_EXT("Master Volume", 2, snd_vol_info, \
			 snd_vol_get, snd_vol_put, 0),
	QSD_EXT("Master Mute", 3, snd_mute_info, \
			 snd_mute_get, snd_mute_put, 0),
};

static int qsd_new_mixer(struct snd_card *card)
{
	unsigned int idx;
	int err;

	strcpy(card->mixername, "MSM Mixer");
	for (idx = 0; idx < ARRAY_SIZE(snd_qsd_controls); idx++) {
		err = snd_ctl_add(card,
				snd_ctl_new1(&snd_qsd_controls[idx], NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int qsd_pcm_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct snd_soc_codec *codec;
	int ret;

	struct snd_soc_device *socdev = platform_get_drvdata(devptr);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	codec->name = "MSM-CARD";
	codec->owner = THIS_MODULE;
	socdev->codec = codec;
	mutex_init(&codec->mutex);

	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "qsd_soc: failed to create pcms\n");
		goto __nopcm;
	}

	card = socdev->codec->card;

	ret = qsd_new_mixer(card);
	if (ret < 0) {
		printk(KERN_ERR "qsd_soc:ALSA MSM Mixer Fail");
		goto __nodev;
	}

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "qsd_soc: failed to register card\n");
		goto __nodev;
	}

	qsd_glb_ctl.playback_device = CAD_HW_DEVICE_ID_DEFAULT_RX;
	qsd_glb_ctl.capture_device = CAD_HW_DEVICE_ID_DEFAULT_TX;
	qsd_glb_ctl.volume = 50;

	return 0;

__nodev:
	snd_soc_free_pcms(socdev);
__nopcm:
	kfree(codec);
	return ret;
}

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
	.probe          = qsd_pcm_probe,
	.remove         = qsd_pcm_remove,
	.pcm_ops 	= &qsd_pcm_ops,
	.pcm_new	= qsd_pcm_new,
};

/* qsd_audio audio subsystem */
static struct snd_soc_device qsd_audio_snd_devdata = {
	.machine = &snd_soc_machine_qsd,
	.platform = &qsd_soc_platform,
};

static int __init qsd_audio_init(void)
{
	int ret;
	struct snd_soc_codec_device *qsd_soc_codec_device;

	qsd_audio_snd_device = platform_device_alloc("soc-audio", -1);
	if (!qsd_audio_snd_device)
		return -ENOMEM;
	qsd_soc_codec_device = kzalloc(sizeof(struct snd_soc_codec_device),
								GFP_KERNEL);
	if (!qsd_soc_codec_device) {
		platform_device_put(qsd_audio_snd_device);
		return -ENOMEM;
	}
	qsd_audio_snd_devdata.codec_dev = qsd_soc_codec_device;

	platform_set_drvdata(qsd_audio_snd_device, &qsd_audio_snd_devdata);
	qsd_audio_snd_devdata.dev = &qsd_audio_snd_device->dev;

	ret = platform_device_add(qsd_audio_snd_device);
	if (ret) {
		platform_device_put(qsd_audio_snd_device);
		return ret;
	}
	mutex_init(&the_locks.lock);
	spin_lock_init(&the_locks.mixer_lock);

	return ret;
}

static void __exit qsd_audio_exit(void)
{
	kfree(qsd_audio_snd_devdata.codec_dev);
	platform_device_unregister(qsd_audio_snd_device);
}

module_init(qsd_audio_init);
module_exit(qsd_audio_exit);

MODULE_DESCRIPTION("PCM module");
MODULE_LICENSE("GPL v2");
