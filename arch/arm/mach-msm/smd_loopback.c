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
 * SMD Loopback Driver -- Provides a loopback SMD port interface.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include <mach/msm_smd.h>

#include "smd_private.h"

#define MAX_BUF_SIZE 512

static DEFINE_MUTEX(loopback_ch_lock);
static DEFINE_MUTEX(loopback_rx_buf_lock);
static DEFINE_MUTEX(loopback_tx_buf_lock);
static DEFINE_SPINLOCK(loopback_read_lock);

static DECLARE_WAIT_QUEUE_HEAD(loopback_wait_queue);

struct loopback_device_t {
	struct miscdevice misc;

	struct smd_channel *ch;

	unsigned char tx_buf[MAX_BUF_SIZE];
	unsigned char rx_buf[MAX_BUF_SIZE];
	unsigned int read_avail;
};

struct loopback_device_t *loopback_devp;

static void loopback_notify(void *priv, unsigned event)
{
	unsigned long flags;

	switch (event) {
	case SMD_EVENT_DATA: {
		int sz;
		sz = smd_cur_packet_size(loopback_devp->ch);
		if ((sz > 0) && (sz <= smd_read_avail(loopback_devp->ch))) {
			/* queue_work(loopback_wq, &loopback_work); */
			spin_lock_irqsave(&loopback_read_lock, flags);
			if (loopback_devp->read_avail == 0) {
				loopback_devp->read_avail = sz;
				wake_up_interruptible(&loopback_wait_queue);
			}
			spin_unlock_irqrestore(&loopback_read_lock, flags);
		}
		break;
	}
	case SMD_EVENT_OPEN:
		printk(KERN_INFO "loopback: smd opened\n");
		break;
	case SMD_EVENT_CLOSE:
		printk(KERN_INFO "loopback: smd closed\n");
		break;
	}
}

static ssize_t loopback_read(struct file *fp, char __user *buf,
			 size_t count, loff_t *pos)
{
	int r = 0;
	int bytes_read = 0;
	unsigned long flags;
	int sz;

	for (;;) {
		mutex_lock(&loopback_rx_buf_lock);

		spin_lock_irqsave(&loopback_read_lock, flags);
		loopback_devp->read_avail = 0;
		spin_unlock_irqrestore(&loopback_read_lock, flags);

		sz = smd_cur_packet_size(loopback_devp->ch);

		if ((sz != 0) && (sz <= smd_read_avail(loopback_devp->ch))) {
			if (sz > MAX_BUF_SIZE) {
				smd_read(loopback_devp->ch, 0, sz);
				mutex_unlock(&loopback_rx_buf_lock);
				continue;
			}

			if (sz != smd_read(loopback_devp->ch,
						loopback_devp->rx_buf, sz)) {
				mutex_unlock(&loopback_rx_buf_lock);
				continue;
			}

			bytes_read = sz;
			break;
		}

		mutex_unlock(&loopback_rx_buf_lock);
		r = wait_event_interruptible(loopback_wait_queue,
						 loopback_devp->read_avail);
		if (r < 0) {
			/* qualify error message */
			if (r != -ERESTARTSYS) {
				/* we get this anytime a signal comes in */
				printk(KERN_ERR "ERROR:%s:%i:%s: "
					"wait_event_interruptible ret %i\n",
					__FILE__,
					__LINE__,
					__func__,
					r
					);
			}
			return r;
		}
	}

	r = copy_to_user(buf, loopback_devp->rx_buf, bytes_read);
	mutex_unlock(&loopback_rx_buf_lock);

	if (r > 0) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
			"copy_to_user could not copy %i bytes.\n",
			__FILE__,
			__LINE__,
			__func__,
			r);
	}

	return bytes_read;
}

static ssize_t loopback_write(struct file *file,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	int r, n;

	if (count > MAX_BUF_SIZE)
		count = MAX_BUF_SIZE;

	r = copy_from_user(loopback_devp->tx_buf, buf, count);
	if (r > 0) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
			"copy_from_user could not copy %i bytes.\n",
			__FILE__,
			__LINE__,
			__func__,
			r);
		return 0;
	}

	mutex_lock(&loopback_tx_buf_lock);
	n = smd_write_avail(loopback_devp->ch);
	while (n < count) {
		mutex_unlock(&loopback_tx_buf_lock);
		msleep(250);
		mutex_lock(&loopback_tx_buf_lock);
		n = smd_write_avail(loopback_devp->ch);
	}

	r = smd_write(loopback_devp->ch, loopback_devp->tx_buf, count);
	mutex_unlock(&loopback_tx_buf_lock);

	if (r != count) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
			"smd_write(ch,buf,count = %i) ret %i.\n",
			__FILE__,
			__LINE__,
			__func__,
			count,
			r);
		return r;
	}

	return count;
}

static int loopback_open(struct inode *ip, struct file *fp)
{
	int r = 0;

	mutex_lock(&loopback_ch_lock);
	if (loopback_devp->ch == 0) {
		smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);
		msleep(100);
		r = smd_open("LOOPBACK", &loopback_devp->ch,
				0, loopback_notify);
	}

	mutex_unlock(&loopback_ch_lock);
	return r;
}

static int loopback_release(struct inode *ip, struct file *fp)
{
	int r = 0;

	mutex_lock(&loopback_ch_lock);
	if (loopback_devp->ch != 0) {
		r = smd_close(loopback_devp->ch);
		loopback_devp->ch = 0;
	}
	mutex_unlock(&loopback_ch_lock);

	return r;
}

static const struct file_operations loopback_fops = {
	.owner = THIS_MODULE,
	.read = loopback_read,
	.write = loopback_write,
	.open = loopback_open,
	.release = loopback_release,
};

static struct loopback_device_t loopback_device = {
	.misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "loopback",
		.fops = &loopback_fops,
	}
};

static void __exit loopback_exit(void)
{
	misc_deregister(&loopback_device.misc);
}

static int __init loopback_init(void)
{
	int ret;

	loopback_devp = &loopback_device;

	ret = misc_register(&loopback_device.misc);
	return ret;
}

module_init(loopback_init);
module_exit(loopback_exit);

MODULE_DESCRIPTION("MSM Shared Memory loopback Driver");
MODULE_LICENSE("GPL v2");
