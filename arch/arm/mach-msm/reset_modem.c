/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
/*
 * MSM architecture driver to reset the modem
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include "smd_private.h"
#include "proc_comm.h"

#define DEBUG
/* #undef DEBUG */
#ifdef DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

static ssize_t reset_modem_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	return 0;
}

static ssize_t reset_modem_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	unsigned char cmd[64];
	int len;
	int time;
	int zero = 0;
	int r;

	if (count < 1)
		return 0;

	len = count > 63 ? 63 : count;

	if (copy_from_user(cmd, buf, len))
		return -EFAULT;

	cmd[len] = 0;

	/* lazy */
	if (cmd[len-1] == '\n') {
		cmd[len-1] = 0;
		len--;
	}

	if (!strncmp(cmd, "wait", 4)) {
		D(KERN_ERR "INFO:%s:%i:%s: "
		       "MODEM RESTART: WAIT\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		smsm_reset_modem(SMSM_MODEM_WAIT);
	} else if (!strncmp(cmd, "continue", 8)) {
		D(KERN_ERR "INFO:%s:%i:%s: "
		       "MODEM RESTART: CONTINUE\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		smsm_reset_modem_cont();
	} else if (!strncmp(cmd, "download", 8)) {
		D(KERN_ERR "INFO:%s:%i:%s: "
		       "MODEM RESTART: DOWNLOAD\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		smsm_reset_modem(SMSM_SYSTEM_DOWNLOAD);
	} else if (sscanf(cmd, "deferred reset %i", &time) == 1) {
		D(KERN_ERR "INFO:%s:%i:%s: "
		       "MODEM RESTART: DEFERRED RESET %ims\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       time);
		if (time == 0) {
			r = 0;
			msm_proc_comm_reset_modem_now();
		} else {
			r = msm_proc_comm(PCOM_RESET_MODEM, &time, &zero);
		}
		if (r < 0)
			return r;
	} else if (!strncmp(cmd, "deferred reset", 14)) {
		D(KERN_ERR "INFO:%s:%i:%s: "
		       "MODEM RESTART: DEFERRED RESET 0ms\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		r = 0;
		msm_proc_comm_reset_modem_now();
		if (r < 0)
			return r;
	} else if (!strncmp(cmd, "reset chip now", 14)) {
		uint param1 = 0x0;
		uint param2 = 0x0;

		D(KERN_ERR "INFO:%s:%i:%s: "
		  "MODEM RESTART: CHIP RESET IMMEDIATE\n",
		  __FILE__,
		  __LINE__,
		  __func__);

		r = msm_proc_comm(PCOM_RESET_CHIP_IMM, &param1, &param2);

		if (r < 0)
			return r;
	} else if (!strncmp(cmd, "reset chip", 10)) {

		uint param1 = 0x0;
		uint param2 = 0x0;

		D(KERN_ERR "INFO:%s:%i:%s: "
		  "MODEM RESTART: CHIP RESET \n",
		  __FILE__,
		  __LINE__,
		  __func__);

		r = msm_proc_comm(PCOM_RESET_CHIP, &param1, &param2);

		if (r < 0)
			return r;
	} else { /* if (!strncmp(cmd, "reset", 5)) */
		printk(KERN_ERR "INFO:%s:%i:%s: "
		       "MODEM RESTART: RESET\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		smsm_reset_modem(SMSM_RESET);
	}

	return count;
}

static int reset_modem_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int reset_modem_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations reset_modem_fops = {
	.owner = THIS_MODULE,
	.read = reset_modem_read,
	.write = reset_modem_write,
	.open = reset_modem_open,
	.release = reset_modem_release,
};

static struct miscdevice reset_modem_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "reset_modem",
	.fops = &reset_modem_fops,
};

static int __init reset_modem_init(void)
{
	return misc_register(&reset_modem_dev);
}

module_init(reset_modem_init);

MODULE_DESCRIPTION("Reset Modem");
MODULE_LICENSE("GPL v2");
