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
#include <linux/msm_audio_aac.h>

#include <asm/ioctls.h>
#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_ard.h>
#include <mach/qdsp6/msm8k_cad_write_aac_format.h>
#include <mach/qdsp6/msm8k_cad_devices.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_aac_in: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_AAC_IN_PROC_NAME "msm8k_aac_in"

#define AUDIO_MAGIC 'a'

struct aac {
	u32 cad_w_handle;
	struct msm_audio_stream_config str_cfg;
	struct msm_audio_aac_enc_config aac_cfg;
};

static int msm8k_aac_in_open(struct inode *inode, struct file *f)
{
	struct aac *aac;
	struct cad_open_struct_type  cos;
	D("%s\n", __func__);

	aac = kmalloc(sizeof(struct aac), GFP_KERNEL);

	if (aac == NULL) {
		pr_err("Could not allocate memory for aac recording driver\n");
		return CAD_RES_FAILURE;
	}

	f->private_data = aac;
	memset(aac, 0, sizeof(struct aac));

	aac->str_cfg.buffer_size = 4096;
	aac->str_cfg.buffer_count = 2;
	aac->aac_cfg.channels = 1;
	aac->aac_cfg.sample_rate = 48000;
	aac->aac_cfg.bit_rate = 192000;
	aac->aac_cfg.stream_format = AUDIO_AAC_FORMAT_ADTS;

	cos.format = CAD_FORMAT_AAC;
	cos.op_code = CAD_OPEN_OP_READ;
	aac->cad_w_handle = cad_open(&cos);

	if (aac->cad_w_handle == 0)
		return CAD_RES_FAILURE;
	else
		return CAD_RES_SUCCESS;
}

static int msm8k_aac_in_release(struct inode *inode, struct file *f)
{
	int rc = CAD_RES_SUCCESS;
	struct aac *aac = f->private_data;
	D("%s\n", __func__);

	cad_close(aac->cad_w_handle);
	kfree(aac);
	return rc;
}

static ssize_t msm8k_aac_in_read(struct file *f, char __user *buf, size_t cnt,
		loff_t *pos)
{
	struct aac			*aac = f->private_data;
	struct cad_buf_struct_type	cbs;

	D("%s\n", __func__);

	memset(&cbs, 0, sizeof(struct cad_buf_struct_type));
	cbs.buffer = (void *)buf;
	cbs.max_size = cnt;
	cbs.actual_size = cnt;


	cnt = cad_read(aac->cad_w_handle, &cbs);
	return cnt;
}

static ssize_t msm8k_aac_in_write(struct file *f, const char __user *buf,
		size_t cnt, loff_t *pos)
{
	D("%s\n", __func__);

	return cnt;
}

static int msm8k_aac_in_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc = CAD_RES_SUCCESS;
	struct aac *p = f->private_data;
	u32 stream_device[1];
	struct cad_device_struct_type cad_dev;
	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_aac_format_struct_type cad_write_aac_fmt;
	D("%s\n", __func__);

	memset(&cad_dev, 0, sizeof(struct cad_device_struct_type));
	memset(&cad_stream_dev, 0,
			sizeof(struct cad_stream_device_struct_type));
	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));
	memset(&cad_write_aac_fmt, 0,
			sizeof(struct cad_write_aac_format_struct_type));

	switch (cmd) {
	case AUDIO_START:

		cad_stream_info.app_type = CAD_STREAM_APP_RECORD;
		cad_stream_info.priority = 0;
		cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
		cad_stream_info.ses_buf_max_size = p->str_cfg.buffer_size;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
			&cad_stream_info,
			sizeof(struct cad_stream_info_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_INFO failed\n");
			break;
		}

		cad_write_aac_fmt.ver_id = CAD_WRITE_AAC_VERSION_10;

		cad_write_aac_fmt.aac.sample_rate = CAD_SAMPLE_RATE_48000;
		cad_write_aac_fmt.aac.channel_config = p->aac_cfg.channels;
		cad_write_aac_fmt.aac.block_formats = p->aac_cfg.stream_format;
		cad_write_aac_fmt.aac.audio_object_type = 2;
		cad_write_aac_fmt.aac.bit_rate = p->aac_cfg.bit_rate;

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_CONFIG,
			&cad_write_aac_fmt,
			sizeof(struct cad_write_aac_format_struct_type));
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
	case AUDIO_GET_STREAM_CONFIG:
		if (copy_to_user((void *)arg, &p->str_cfg,
				sizeof(struct msm_audio_stream_config)))
			return -EFAULT;
		break;
	case AUDIO_SET_STREAM_CONFIG:
		if (copy_from_user(&p->str_cfg, (void *)arg,
				sizeof(struct msm_audio_stream_config)))
			return -EFAULT;
		break;
	case AUDIO_GET_AAC_ENC_CONFIG:
		if (copy_to_user((void *)arg, &p->aac_cfg,
				sizeof(struct msm_audio_aac_enc_config)))
			return -EFAULT;
		break;
	case AUDIO_SET_AAC_ENC_CONFIG:
		if (copy_from_user(&p->aac_cfg, (void *)arg,
				sizeof(struct msm_audio_aac_enc_config)))
			return -EFAULT;
		if (p->aac_cfg.channels != 1) {
			pr_err("only mono is supported\n");
			return -EINVAL;
		}
		if (p->aac_cfg.sample_rate != 48000) {
			pr_err("only 48KHz is supported\n");
			return -EINVAL;
		}
		if (p->aac_cfg.stream_format != AUDIO_AAC_FORMAT_RAW &&
			p->aac_cfg.stream_format != AUDIO_AAC_FORMAT_ADTS) {
			pr_err("unsupported AAC format\n");
			return -EINVAL;
		}
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_aac_in_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "aac_in\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_aac_in_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_aac_in_open,
	.release = msm8k_aac_in_release,
	.read = msm8k_aac_in_read,
	.write = msm8k_aac_in_write,
	.ioctl = msm8k_aac_in_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_aac_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_aac_in",
	.fops	= &msm8k_aac_in_fops,
};

static int __init msm8k_aac_in_init(void)
{
	int rc;
	D("%s\n", __func__);

	rc = misc_register(&msm8k_aac_in_misc);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_AAC_IN_PROC_NAME,
			0, NULL, msm8k_aac_in_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_aac_in_exit(void)
{
	D("%s\n", __func__);
#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_AAC_IN_PROC_NAME, NULL);
#endif
}


module_init(msm8k_aac_in_init);
module_exit(msm8k_aac_in_exit);

MODULE_DESCRIPTION("MSM AAC recording driver");
MODULE_LICENSE("Dual BSD/GPL");
