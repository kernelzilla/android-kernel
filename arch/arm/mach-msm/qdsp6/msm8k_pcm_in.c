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
#include <mach/qdsp6/msm8k_cad_write_pcm_format.h>
#include <mach/qdsp6/msm8k_cad_devices.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_pcm_in: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_PCM_IN_PROC_NAME "msm8k_pcm_in"

#define AUDIO_MAGIC 'a'

struct pcm {
	u32 cad_w_handle;
	struct msm_audio_config cfg;
	struct msm_voicerec_mode voicerec_mode;
};


static int msm8k_pcm_in_open(struct inode *inode, struct file *f)
{
	struct pcm *pcm;
	struct cad_open_struct_type  cos;
	D("%s\n", __func__);

	pcm = kmalloc(sizeof(struct pcm), GFP_KERNEL);
	if (pcm == NULL) {
		pr_err("Could not allocate memory for pcm_in driver\n");
		return CAD_RES_FAILURE;
	}

	f->private_data = pcm;

	memset(pcm, 0, sizeof(struct pcm));
	pcm->cfg.buffer_size = 4096;
	pcm->cfg.buffer_count = 2;
	pcm->cfg.channel_count = 1;
	pcm->cfg.sample_rate = 8000;
	pcm->voicerec_mode.rec_mode = VOC_REC_UPLINK;

	cos.format = CAD_FORMAT_PCM;
	cos.op_code = CAD_OPEN_OP_READ;
	pcm->cad_w_handle = cad_open(&cos);

	if (pcm->cad_w_handle == 0)
		return CAD_RES_FAILURE;
	else
		return CAD_RES_SUCCESS;
}

static int msm8k_pcm_in_release(struct inode *inode, struct file *f)
{
	int rc = CAD_RES_SUCCESS;
	struct pcm *pcm = f->private_data;
	D("%s\n", __func__);

	cad_close(pcm->cad_w_handle);
	kfree(pcm);

	return rc;
}

static ssize_t msm8k_pcm_in_read(struct file *f, char __user *buf, size_t cnt,
		loff_t *pos)
{
	struct pcm			*pcm = f->private_data;
	struct cad_buf_struct_type	cbs;

	D("%s\n", __func__);

	memset(&cbs, 0, sizeof(struct cad_buf_struct_type));
	cbs.buffer = (void *)buf;
	cbs.max_size = cnt;
	cbs.actual_size = cnt;


	cnt = cad_read(pcm->cad_w_handle, &cbs);
	return cnt;
}

static ssize_t msm8k_pcm_in_write(struct file *f, const char __user *buf,
		size_t cnt, loff_t *pos)
{
	D("%s\n", __func__);

	return cnt;
}

static int msm8k_pcm_in_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc = CAD_RES_SUCCESS;
	struct pcm *p = f->private_data;
	u32 stream_device[1];
	struct cad_device_struct_type cad_dev;
	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_pcm_format_struct_type cad_write_pcm_fmt;
	D("%s\n", __func__);

	memset(&cad_dev, 0, sizeof(struct cad_device_struct_type));
	memset(&cad_stream_dev, 0,
			sizeof(struct cad_stream_device_struct_type));
	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));
	memset(&cad_write_pcm_fmt, 0,
			sizeof(struct cad_write_pcm_format_struct_type));

	switch (cmd) {
	case AUDIO_START:

		if (p->voicerec_mode.rec_mode == VOC_REC_BOTH)
			cad_stream_info.app_type = CAD_STREAM_APP_MIXED_RECORD;
		else
			cad_stream_info.app_type = CAD_STREAM_APP_RECORD;

		cad_stream_info.priority = 0;
		cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
		cad_stream_info.ses_buf_max_size = p->cfg.buffer_size;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
			&cad_stream_info,
			sizeof(struct cad_stream_info_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_INFO failed\n");
			break;
		}

		cad_write_pcm_fmt.us_ver_id = CAD_WRITE_PCM_VERSION_10;
		cad_write_pcm_fmt.pcm.us_channel_config = p->cfg.channel_count;
		cad_write_pcm_fmt.pcm.us_width = 1;
		cad_write_pcm_fmt.pcm.us_sign = 0;

		switch (p->cfg.sample_rate) {
		case 96000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 0;
			break;
		case 88200:
			cad_write_pcm_fmt.pcm.us_sample_rate = 1;
			break;
		case 64000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 2;
			break;
		case 48000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 3;
			break;
		case 44100:
			cad_write_pcm_fmt.pcm.us_sample_rate = 4;
			break;
		case 32000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 5;
			break;
		case 24000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 6;
			break;
		case 22050:
			cad_write_pcm_fmt.pcm.us_sample_rate = 7;
			break;
		case 16000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 8;
			break;
		case 12000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 9;
			break;
		case 11025:
			cad_write_pcm_fmt.pcm.us_sample_rate = 10;
			break;
		case 8000:
			cad_write_pcm_fmt.pcm.us_sample_rate = 11;
			break;
		}

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_CONFIG,
			&cad_write_pcm_fmt,
			sizeof(struct cad_write_pcm_format_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_CONFIG failed\n");
			break;
		}

		stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_TX;
		cad_stream_dev.device = (u32 *)&stream_device[0];
		cad_stream_dev.device_len = 1;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
			&cad_stream_dev,
			sizeof(struct cad_stream_device_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_DEVICE failed\n");
			break;
		}

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
			NULL, 0);
		if (rc) {
			pr_err("cad_ioctl() STREAM_START failed\n");
			break;
		}
		break;
	case AUDIO_STOP:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_PAUSE,
			NULL, 0);
		break;
	case AUDIO_FLUSH:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_FLUSH,
			NULL, 0);
		break;
	case AUDIO_GET_CONFIG:
		if (copy_to_user((void *)arg, &p->cfg,
				sizeof(struct msm_audio_config)))
			return -EFAULT;
		break;
	case AUDIO_SET_CONFIG:
		rc = copy_from_user(&p->cfg, (void *)arg,
				sizeof(struct msm_audio_config));
		break;
	case AUDIO_SET_INCALL:
		if (copy_from_user(&p->voicerec_mode, (void *)arg,
				sizeof(struct msm_voicerec_mode)))
			return -EFAULT;
		if (p->voicerec_mode.rec_mode != VOC_REC_BOTH &&
			p->voicerec_mode.rec_mode != VOC_REC_UPLINK &&
			p->voicerec_mode.rec_mode != VOC_REC_DOWNLINK) {
			p->voicerec_mode.rec_mode = VOC_REC_UPLINK;
			pr_err("invalid rec_mode\n");
			return -EINVAL;
		}
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_pcm_in_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "pcm_in\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_pcm_in_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_pcm_in_open,
	.release = msm8k_pcm_in_release,
	.read = msm8k_pcm_in_read,
	.write = msm8k_pcm_in_write,
	.ioctl = msm8k_pcm_in_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_pcm_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_in",
	.fops	= &msm8k_pcm_in_fops,
};

static int __init msm8k_pcm_in_init(void)
{
	int rc;
	D("%s\n", __func__);

	rc = misc_register(&msm8k_pcm_in_misc);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_PCM_IN_PROC_NAME,
			0, NULL, msm8k_pcm_in_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_pcm_in_exit(void)
{
	D("%s\n", __func__);
#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_PCM_IN_PROC_NAME, NULL);
#endif
}


module_init(msm8k_pcm_in_init);
module_exit(msm8k_pcm_in_exit);

MODULE_DESCRIPTION("MSM PCM IN driver");
MODULE_LICENSE("GPL v2");

