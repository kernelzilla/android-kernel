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
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>

#include <asm/ioctls.h>
#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_ard.h>
#include <mach/qdsp6/msm8k_cad_write_aac_format.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_volume.h>
#include <linux/msm_audio_aac.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_aac: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_AAC_PROC_NAME "msm8k_aac"

#define AUDIO_MAGIC 'a'

struct aac {
	u32 cad_w_handle;
	u32 volume;
	struct msm_audio_config cfg;
	struct msm_audio_aac_config aac_cfg;
};

struct aac g_aac;

static int msm8k_aac_open(struct inode *inode, struct file *f)
{
	struct aac *aac = &g_aac;
	struct cad_open_struct_type  cos;
	D("%s\n", __func__);

	cos.format = CAD_FORMAT_AAC;

	f->private_data = aac;

	cos.op_code = CAD_OPEN_OP_WRITE;
	aac->cad_w_handle = cad_open(&cos);

	if (aac->cad_w_handle == 0)
		return CAD_RES_FAILURE;
	else
		return CAD_RES_SUCCESS;
}

static int msm8k_aac_release(struct inode *inode, struct file *f)
{
	struct aac *aac = f->private_data;
	D("%s\n", __func__);

	cad_close(aac->cad_w_handle);

	return CAD_RES_SUCCESS;
}

static ssize_t msm8k_aac_read(struct file *f, char __user *buf, size_t cnt,
		loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

static ssize_t msm8k_aac_write(struct file *f, const char __user *buf,
		size_t cnt, loff_t *pos)
{
	struct cad_buf_struct_type cbs;
	struct aac *aac = f->private_data;

	D("%s\n", __func__);

	memset(&cbs, 0, sizeof(struct cad_buf_struct_type));
	cbs.buffer = (void *)buf;
	cbs.phys_addr = 0;
	cbs.max_size = cnt;

	cad_write(aac->cad_w_handle, &cbs);

	return cnt;
}

static int msm8k_aac_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc;
	struct aac *p = f->private_data;
	void *cad_arg = (void *)arg;
	u32 stream_device[1];
	struct cad_device_struct_type cad_dev;
	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_aac_format_struct_type cad_write_aac_fmt;
	struct cad_flt_cfg_strm_vol cad_strm_volume;
	struct cad_stream_filter_struct_type cad_stream_filter;
	struct msm_audio_aac_config ncfg;

	D("%s\n", __func__);
	memset(&cad_dev, 0, sizeof(struct cad_device_struct_type));
	memset(&cad_stream_dev, 0,
			sizeof(struct cad_stream_device_struct_type));
	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));
	memset(&cad_write_aac_fmt, 0,
			sizeof(struct cad_write_aac_format_struct_type));
	memset(&cad_stream_filter, 0,
			sizeof(struct cad_stream_filter_struct_type));

	switch (cmd) {
	case AUDIO_START:

		cad_stream_info.app_type = CAD_STREAM_APP_PLAYBACK;
		cad_stream_info.priority = 0;
		cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
		cad_stream_info.ses_buf_max_size = 1024 * 10;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
			&cad_stream_info,
			sizeof(struct cad_stream_info_struct_type));
		if (rc) {
			D("cad_ioctl() SET_STREAM_INFO failed\n");
			break;
		}

		stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_RX;
		cad_stream_dev.device = (u32 *)&stream_device[0];
		cad_stream_dev.device_len = 1;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
			&cad_stream_dev,
			sizeof(struct cad_stream_device_struct_type));
		if (rc) {
			D("cad_ioctl() SET_STREAM_DEVICE failed\n");
			break;
		}

		cad_write_aac_fmt.ver_id = CAD_WRITE_AAC_VERSION_10;
		switch (p->cfg.sample_rate) {
		case 96:
		case 96000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_96000;
			break;
		case 88:
		case 88200:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_88200;
			break;
		case 64:
		case 64000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_64000;
			break;
		case 48:
		case 48000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_48000;
			break;
		case 44:
		case 44100:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_44100;
			break;
		case 32:
		case 32000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_32000;
			break;
		case 24:
		case 24000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_24000;
			break;
		case 22:
		case 22050:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_22050;
			break;
		case 16:
		case 16000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_16000;
			break;
		case 12:
		case 12000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_12000;
			break;
		case 11:
		case 11025:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_11025;
			break;
		case 8:
		case 8000:
			cad_write_aac_fmt.aac.sample_rate
						= CAD_SAMPLE_RATE_08000;
			break;
		default:
			return -EINVAL;
		}
		cad_write_aac_fmt.aac.channel_config =
			p->aac_cfg.channel_configuration;
		cad_write_aac_fmt.aac.block_formats = p->aac_cfg.format;
		cad_write_aac_fmt.aac.audio_object_type
			= p->aac_cfg.audio_object;
		cad_write_aac_fmt.aac.ep_config =
			p->aac_cfg.ep_config;
		cad_write_aac_fmt.aac.aac_section_data_resilience_flag =
			p->aac_cfg.aac_section_data_resilience_flag;
		cad_write_aac_fmt.aac.aac_scalefactor_data_resilience_flag =
			p->aac_cfg.aac_scalefactor_data_resilience_flag;
		cad_write_aac_fmt.aac.aac_spectral_data_resilience_flag =
			p->aac_cfg.aac_spectral_data_resilience_flag;
#ifndef CONFIG_AUDIO_AAC_PLUS
		cad_write_aac_fmt.aac.sbr_on_flag = AUDIO_AAC_SBR_ON_FLAG_OFF;
#else
		cad_write_aac_fmt.aac.sbr_on_flag = p->aac_cfg.sbr_on_flag;
#endif
#ifndef CONFIG_AUDIO_ENHANCED_AAC_PLUS
		cad_write_aac_fmt.aac.sbr_ps_on_flag =
			AUDIO_AAC_SBR_PS_ON_FLAG_OFF;
#else
		cad_write_aac_fmt.aac.sbr_ps_on_flag =
			p->aac_cfg.sbr_ps_on_flag;
#endif

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_CONFIG,
			&cad_write_aac_fmt,
			sizeof(struct cad_write_aac_format_struct_type));
		if (rc) {
			D("cad_ioctl() SET_STREAM_CONFIG failed\n");
			break;
		}

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
			NULL, 0);
		if (rc) {
			D("cad_ioctl() STREAM_START failed\n");
			break;
		}
		break;
	case AUDIO_STOP:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_PAUSE,
			cad_arg, sizeof(u32));
		break;
	case AUDIO_FLUSH:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_FLUSH,
			cad_arg, sizeof(u32));
		break;
	case AUDIO_GET_PCM_CONFIG:
		D("AUDIO_GET_PCM_CONFIG\n");
		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_SET_PCM_CONFIG:
		D("AUDIO_SET_PCM_CONFIG\n");
		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_GET_CONFIG:
		if (copy_to_user((void *)arg, &p->cfg,
				sizeof(struct msm_audio_config)))
			return -EFAULT;
		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_SET_CONFIG:
		rc = copy_from_user(&p->cfg, (void *)arg,
				sizeof(struct msm_audio_config));
		if (rc)
			D("cad_ioctl() SET_STREAM_CONFIG failed\n");
		break;
	case AUDIO_GET_AAC_CONFIG:
#ifndef CONFIG_AUDIO_AAC_PLUS
		p->aac_cfg.sbr_on_flag = AUDIO_AAC_SBR_ON_FLAG_OFF;
#else
		p->aac_cfg.sbr_on_flag = AUDIO_AAC_SBR_ON_FLAG_ON;
#endif
#ifndef CONFIG_AUDIO_ENHANCED_AAC_PLUS
		p->aac_cfg.sbr_ps_on_flag = AUDIO_AAC_SBR_PS_ON_FLAG_OFF;
#else
		p->aac_cfg.sbr_ps_on_flag = AUDIO_AAC_SBR_PS_ON_FLAG_ON;
#endif
		if (copy_to_user((void *)arg, &p->aac_cfg,
				sizeof(struct msm_audio_aac_config)))
			return -EFAULT;
		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_SET_AAC_CONFIG:
		if (copy_from_user(&ncfg, (void *)arg,
				sizeof(struct msm_audio_aac_config)))
			return -EFAULT;
		p->aac_cfg.channel_configuration = ncfg.channel_configuration;
		p->aac_cfg.format = ncfg.format;
		p->aac_cfg.ep_config = ncfg.ep_config;
		p->aac_cfg.aac_section_data_resilience_flag =
				ncfg.aac_section_data_resilience_flag;
		p->aac_cfg.aac_scalefactor_data_resilience_flag =
				ncfg.aac_scalefactor_data_resilience_flag;
		p->aac_cfg.aac_spectral_data_resilience_flag =
				ncfg.aac_spectral_data_resilience_flag;
#ifndef CONFIG_AUDIO_AAC_PLUS
		p->aac_cfg.sbr_on_flag = AUDIO_AAC_SBR_ON_FLAG_OFF;
#else
		p->aac_cfg.sbr_on_flag = ncfg.sbr_on_flag;
#endif
#ifndef CONFIG_AUDIO_ENHANCED_AAC_PLUS
		p->aac_cfg.sbr_ps_on_flag = AUDIO_AAC_SBR_PS_ON_FLAG_OFF;
#else
		p->aac_cfg.sbr_ps_on_flag = ncfg.sbr_ps_on_flag;
#endif
		if ((ncfg.audio_object == CAD_AUDIO_OBJ_TYPE_AAC_LC)
			|| (ncfg.audio_object == CAD_AUDIO_OBJ_TYPE_AAC_LTP)
			|| (ncfg.audio_object == CAD_AUDIO_OBJ_TYPE_ER_AAC_LC)
			|| (ncfg.audio_object == CAD_AUDIO_OBJ_TYPE_AAC_BSAC))
			p->aac_cfg.audio_object = ncfg.audio_object;

		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_SET_VOLUME:
		rc = copy_from_user(&p->volume, (void *)arg, sizeof(u32));

		memset(&cad_strm_volume, 0,
				sizeof(struct cad_flt_cfg_strm_vol));
		cad_strm_volume.volume = p->volume;
		cad_stream_filter.filter_type = CAD_FILTER_CONFIG_STREAM_VOLUME;
		cad_stream_filter.format_block = &cad_strm_volume;
		cad_stream_filter.format_block_len =
			sizeof(struct cad_flt_cfg_strm_vol);

		rc = cad_ioctl(p->cad_w_handle,
			CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG,
			&cad_stream_filter,
			sizeof(struct cad_stream_filter_struct_type));
		if (rc) {
			D("cad_ioctl() set volume failed\n");
			break;
		}
		break;
	default:
		D("unknown ioctl: 0x%08X\n", cmd);
		rc = CAD_RES_SUCCESS;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_aac_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "aac\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_aac_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_aac_open,
	.release = msm8k_aac_release,
	.read = msm8k_aac_read,
	.write = msm8k_aac_write,
	.ioctl = msm8k_aac_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_aac_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_aac",
	.fops	= &msm8k_aac_fops,
};

static int __init msm8k_aac_init(void)
{
	int rc;
	D("%s\n", __func__);

	memset(&g_aac, 0, sizeof(struct aac));

	g_aac.cfg.buffer_size = 4096;
	g_aac.cfg.buffer_count = 2;
	g_aac.cfg.channel_count = 1;
	/* This sample rate will be converted to the CAD value upon start. */
	g_aac.cfg.sample_rate = 48000;

	g_aac.aac_cfg.format = CAD_BLK_FMT_ADTS;
	g_aac.aac_cfg.audio_object = CAD_AUDIO_OBJ_TYPE_AAC_LC;
	g_aac.aac_cfg.ep_config = CAD_ERR_PROT_SCHEME_0;
	g_aac.aac_cfg.channel_configuration = CAD_CHANNEL_CFG_MONO;

	rc = misc_register(&msm8k_aac_misc);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_AAC_PROC_NAME,
			0, NULL, msm8k_aac_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_aac_exit(void)
{
	D("%s\n", __func__);
#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_AAC_PROC_NAME, NULL);
#endif
}


module_init(msm8k_aac_init);
module_exit(msm8k_aac_exit);

MODULE_DESCRIPTION("MSM AAC driver");
MODULE_LICENSE("GPL v2");

