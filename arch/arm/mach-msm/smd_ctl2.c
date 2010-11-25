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
 * SMD Control Driver -- Provides a binary SMD non-muxed control port
 *                       interface.
 */

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <asm/ioctls.h>

#include <mach/msm_smd.h>

#include "modem_notifier.h"

#define NUM_SMD_CTL_PORTS 3
#define DEVICE_NAME "smdcntl"
#define MAX_BUF_SIZE 2048

struct smd_ctl_dev {
	struct cdev cdev;
	char name[9];
	struct device *devicep;

	struct smd_channel *ctl_ch;
	struct mutex ctl_ch_lock;
	struct mutex rx_lock;
	struct mutex is_open_lock;
	struct workqueue_struct *ctl_wq;
	struct work_struct ctl_work;
	wait_queue_head_t ctl_wait_queue;
	wait_queue_head_t ctl_opened_wait_queue;

	int i;

	unsigned char tx_buf[MAX_BUF_SIZE];
	unsigned char rx_buf[MAX_BUF_SIZE];
	int bytes_read;
	int is_open;

	struct notifier_block nb;
	int has_reset;
	struct mutex has_reset_lock;

} *smd_ctl_devp[NUM_SMD_CTL_PORTS];

struct class *smd_ctl_classp;
static dev_t smd_ctl_number;

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define D_DUMP_BUFFER(prestr, cnt, buf) \
do { \
	int i; \
	printk(KERN_ERR "%s", prestr); \
	for (i = 0; i < cnt; i++) \
		printk(KERN_ERR "%.2x", buf[i]); \
	printk(KERN_ERR "\n"); \
} while (0)
#else
#define D_DUMP_BUFFER(prestr, cnt, buf) do {} while (0)
#endif

#ifdef DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

static void clean_and_signal(struct smd_ctl_dev *smd_ctl_devp)
{
	flush_workqueue(smd_ctl_devp->ctl_wq);

	mutex_lock(&smd_ctl_devp->has_reset_lock);
	smd_ctl_devp->has_reset = 1;
	mutex_unlock(&smd_ctl_devp->has_reset_lock);

	mutex_lock(&smd_ctl_devp->rx_lock);
	smd_ctl_devp->bytes_read = 0;
	mutex_unlock(&smd_ctl_devp->rx_lock);

	mutex_lock(&smd_ctl_devp->is_open_lock);
	smd_ctl_devp->is_open = 0;
	mutex_unlock(&smd_ctl_devp->is_open_lock);

	wake_up_interruptible(&smd_ctl_devp->ctl_wait_queue);
	wake_up_interruptible(&smd_ctl_devp->ctl_opened_wait_queue);
}

static int modem_notifier(struct notifier_block *this,
			  unsigned long code,
			  void *_cmd)
{
	struct smd_ctl_dev *smd_ctl_devp =
		container_of(this,
			     struct smd_ctl_dev,
			     nb);

	if (!smd_ctl_devp)
		return NOTIFY_DONE;

	switch (code) {
	case MODEM_NOTIFIER_START_RESET:
		printk(KERN_ERR "Notify: start reset ch:%i\n",
		       smd_ctl_devp->i);
		clean_and_signal(smd_ctl_devp);
		break;
	case MODEM_NOTIFIER_END_RESET:
		printk(KERN_ERR "Notify: end reset\n");
		break;
	default:
		printk(KERN_ERR "Notify: general\n");
		break;
	}
	return NOTIFY_DONE;
}

int smd_ctl_ioctl(struct inode *inode,
		    struct file *file,
		    unsigned int cmd,
		    unsigned long arg)
{
	int ret;
	struct smd_ctl_dev *smd_ctl_devp;

	smd_ctl_devp = file->private_data;

	switch (cmd) {
	case TIOCMGET:
		ret = smd_tiocmget(smd_ctl_devp->ctl_ch);
		break;
	case TIOCMSET:
		ret = smd_tiocmset(smd_ctl_devp->ctl_ch, arg, ~arg);
		break;
	default:
		ret = -1;
	}

	return ret;
}

ssize_t smd_ctl_read(struct file *file,
		       char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	int r;
	int bytes_read;
	struct smd_ctl_dev *smd_ctl_devp;

	D(KERN_ERR "%s: read %i bytes\n",
	  __func__, count);

	smd_ctl_devp = file->private_data;

	if (!smd_ctl_devp->ctl_ch)
		return -EINVAL;

	r = wait_event_interruptible(smd_ctl_devp->ctl_wait_queue,
				     smd_ctl_devp->bytes_read |
				     smd_ctl_devp->has_reset);

	if (smd_ctl_devp->has_reset)
		return -ENETRESET;

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

	/* Here we have a whole packet waiting for us */

	mutex_lock(&smd_ctl_devp->rx_lock);
	bytes_read = smd_ctl_devp->bytes_read;
	smd_ctl_devp->bytes_read = 0;
	mutex_unlock(&smd_ctl_devp->rx_lock);

	D(KERN_ERR "%s: after wait_event_interruptible bytes_read = %i\n",
	  __func__, bytes_read);

	if (bytes_read > count) {
		printk(KERN_ERR "packet size %i > buffer size %i, "
		       "dropping packet!", bytes_read, count);
		smd_read(smd_ctl_devp->ctl_ch, 0, bytes_read);
		return -EINVAL;
	}

	/* smd_read and copy_to_user need to be merged to only do 1 copy */
	if (smd_read(smd_ctl_devp->ctl_ch, smd_ctl_devp->rx_buf, bytes_read)
	    != bytes_read) {
		if (smd_ctl_devp->has_reset)
			return -ENETRESET;

		printk(KERN_ERR "user read: not enough data?!\n");
		return -EINVAL;
	}
	D_DUMP_BUFFER("read: ", bytes_read, smd_ctl_devp->rx_buf);
	r = copy_to_user(buf, smd_ctl_devp->rx_buf, bytes_read);

	if (r > 0) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "copy_to_user could not copy %i bytes.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       r);
		return r;
	}

	D(KERN_ERR "%s: just read %i bytes\n",
	  __func__, bytes_read);

	/* Not all packet events get explictly handled, this doesn't
	   matter if a constant stream of packets is streaming in, but
	   eventually a packet will be received and we'll have missed
	   the event. Queuing one more work item will catch this if
	   its happened, but do nothing if it hasn't.
	*/
	queue_work(smd_ctl_devp->ctl_wq, &smd_ctl_devp->ctl_work);

	D(KERN_ERR "%s: just queued more work\n", __func__);

	return bytes_read;
}

ssize_t smd_ctl_write(struct file *file,
		       const char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	int r;
	struct smd_ctl_dev *smd_ctl_devp;

	if (count > MAX_BUF_SIZE)
		return -EINVAL;

	D(KERN_ERR "%s: writting %i bytes\n",
	  __func__, count);

	smd_ctl_devp = file->private_data;

	if (!smd_ctl_devp->ctl_ch)
		return -EINVAL;

	r = wait_event_interruptible(smd_ctl_devp->ctl_opened_wait_queue,
				     smd_ctl_devp->is_open |
				     smd_ctl_devp->has_reset);

	if (smd_ctl_devp->has_reset)
		return -ENETRESET;

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

	D_DUMP_BUFFER("write: ", count, buf);

	r = copy_from_user(smd_ctl_devp->tx_buf, buf, count);
	if (r > 0) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "copy_from_user could not copy %i bytes.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       r);
		return r;
	}

	D(KERN_ERR "%s: after copy_from_user. count = %i\n",
	  __func__, count);

	r = smd_write(smd_ctl_devp->ctl_ch, smd_ctl_devp->tx_buf, count);
	if (r != count) {
		if (smd_ctl_devp->has_reset)
			return -ENETRESET;

		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "smd_write(ch,buf,count = %i) ret %i.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       count,
		       r);
		return r;
	}

	D(KERN_ERR "%s: just wrote %i bytes\n",
	       __func__, count);

	return count;
}

static void ctl_work_func(struct work_struct *work)
{
	/* unsigned char buf[MAX_BUF_SIZE]; */
	int sz;
	struct smd_ctl_dev *smd_ctl_devp = container_of(work,
							struct smd_ctl_dev,
							ctl_work);

	if (!smd_ctl_devp->ctl_ch)
		return;

	for (;;) {
		sz = smd_cur_packet_size(smd_ctl_devp->ctl_ch);
		if (sz == 0) {
			D(KERN_ERR "%s: packet size is 0\n", __func__);
			break;
		}
		if (sz > smd_read_avail(smd_ctl_devp->ctl_ch)) {
			D(KERN_ERR "%s: packet size is %i - "
			  "the whole packet isn't here\n",
			  __func__, sz);
			break;
		}
		if (sz > MAX_BUF_SIZE) {
			smd_read(smd_ctl_devp->ctl_ch, 0, sz);
			D(KERN_ERR "%s: packet size is %i - "
			  "greater than max %i, dropping\n",
			  __func__, sz, MAX_BUF_SIZE);
			continue;
		}

		/* here we have a packet of size sz ready */

		mutex_lock(&smd_ctl_devp->rx_lock);
		smd_ctl_devp->bytes_read = sz;
		mutex_unlock(&smd_ctl_devp->rx_lock);
		wake_up_interruptible(&smd_ctl_devp->ctl_wait_queue);
		D(KERN_ERR "%s: after wake_up\n", __func__);
		break;
	}
}

static void ctl_notify(void *priv, unsigned event)
{
	struct smd_ctl_dev *smd_ctl_devp = priv;

	if (smd_ctl_devp->ctl_ch == 0)
		return;

	switch (event) {
	case SMD_EVENT_DATA: {
		int sz;
		D(KERN_ERR "%s: data\n",
		  __func__);
		sz = smd_cur_packet_size(smd_ctl_devp->ctl_ch);
		D(KERN_ERR "%s: data sz = %i\n",
		  __func__, sz);
		D(KERN_ERR "%s: smd_read_avail = %i\n",
		  __func__, smd_read_avail(smd_ctl_devp->ctl_ch));
		if ((sz > 0) && (sz <= smd_read_avail(smd_ctl_devp->ctl_ch))) {
			queue_work(smd_ctl_devp->ctl_wq,
				   &smd_ctl_devp->ctl_work);
			D(KERN_ERR "%s: data just queued\n",
			  __func__);
		}
		D(KERN_ERR "%s: data after queueing\n",
		  __func__);
		break;
	}
	case SMD_EVENT_OPEN:
		D(KERN_ERR "%s: smd opened\n",
		  __func__);
		smd_ctl_devp->is_open = 1;
		wake_up_interruptible(&smd_ctl_devp->ctl_opened_wait_queue);
		break;
	case SMD_EVENT_CLOSE:
		smd_ctl_devp->is_open = 0;
		printk(KERN_ERR "%s: smd closed\n",
		       __func__);
		break;
	}
}

static char *smd_ctl_name[] = {
	"DATA5_CNTL",
	"DATA6_CNTL",
	"DATA7_CNTL",
};

int smd_ctl_open(struct inode *inode, struct file *file)
{
	int r = 0;
	struct smd_ctl_dev *smd_ctl_devp;

	smd_ctl_devp = container_of(inode->i_cdev, struct smd_ctl_dev, cdev);

	if (!smd_ctl_devp)
		return -EINVAL;

	file->private_data = smd_ctl_devp;

	mutex_lock(&smd_ctl_devp->ctl_ch_lock);
	if (smd_ctl_devp->ctl_ch == 0)
		r = smd_open(smd_ctl_name[smd_ctl_devp->i],
			     &smd_ctl_devp->ctl_ch,
			     smd_ctl_devp,
			     ctl_notify);
	mutex_unlock(&smd_ctl_devp->ctl_ch_lock);

	return r;
}

int smd_ctl_release(struct inode *inode, struct file *file)
{
	int r = 0;
	struct smd_ctl_dev *smd_ctl_devp = file->private_data;

	if (!smd_ctl_devp)
		return -EINVAL;

	clean_and_signal(smd_ctl_devp);

	mutex_lock(&smd_ctl_devp->ctl_ch_lock);
	if (smd_ctl_devp->ctl_ch != 0) {
		r = smd_close(smd_ctl_devp->ctl_ch);
		smd_ctl_devp->ctl_ch = 0;
	}
	mutex_unlock(&smd_ctl_devp->ctl_ch_lock);

	mutex_lock(&smd_ctl_devp->has_reset_lock);
	smd_ctl_devp->has_reset = 0;
	mutex_unlock(&smd_ctl_devp->has_reset_lock);

	return r;
}

static const struct file_operations smd_ctl_fops = {
	.owner = THIS_MODULE,
	.open = smd_ctl_open,
	.release = smd_ctl_release,
	.read = smd_ctl_read,
	.write = smd_ctl_write,
	.ioctl = smd_ctl_ioctl,
};

static int __init smd_ctl_init(void)
{
	int i;
	int r;
	unsigned char buf[32];

	r = alloc_chrdev_region(&smd_ctl_number,
			       0,
			       NUM_SMD_CTL_PORTS,
			       DEVICE_NAME);
	if (IS_ERR_VALUE(r)) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "alloc_chrdev_region() ret %i.\n",
		       __FILE__,
		       __LINE__,
		       __func__,
		       r);
		goto error0;
	}

	smd_ctl_classp = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(smd_ctl_classp)) {
		printk(KERN_ERR "ERROR:%s:%i:%s: "
		       "class_create() ENOMEM\n",
		       __FILE__,
		       __LINE__,
		       __func__);
		r = -ENOMEM;
		goto error1;
	}

	for (i = 0; i < NUM_SMD_CTL_PORTS; ++i) {
		smd_ctl_devp[i] = kzalloc(sizeof(struct smd_ctl_dev),
					 GFP_KERNEL);
		if (IS_ERR(smd_ctl_devp[i])) {
			printk(KERN_ERR "ERROR:%s:%i:%s kmalloc() ENOMEM\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			goto error2;
		}

		smd_ctl_devp[i]->i = i;

		scnprintf(buf, 32, "ctl%i", i);
		smd_ctl_devp[i]->ctl_wq = create_singlethread_workqueue(buf);
		if (&smd_ctl_devp[i]->ctl_wq == 0) {
			printk(KERN_ERR
			       "%s:%i:%s: "
			       "create_singlethread_workqueue() ret 0\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			goto error2;
		}

		init_waitqueue_head(&smd_ctl_devp[i]->ctl_wait_queue);
		smd_ctl_devp[i]->is_open = 0;
		init_waitqueue_head(&smd_ctl_devp[i]->ctl_opened_wait_queue);
		INIT_WORK(&smd_ctl_devp[i]->ctl_work,
			  ctl_work_func);

		mutex_init(&smd_ctl_devp[i]->ctl_ch_lock);
		mutex_init(&smd_ctl_devp[i]->rx_lock);
		mutex_init(&smd_ctl_devp[i]->is_open_lock);

		cdev_init(&smd_ctl_devp[i]->cdev, &smd_ctl_fops);
		smd_ctl_devp[i]->cdev.owner = THIS_MODULE;

		r = cdev_add(&smd_ctl_devp[i]->cdev,
			     (smd_ctl_number + i),
			     1);

		if (IS_ERR_VALUE(r)) {
			printk(KERN_ERR "%s:%i:%s: cdev_add() ret %i\n",
			       __FILE__,
			       __LINE__,
			       __func__,
			       r);
			destroy_workqueue(smd_ctl_devp[i]->ctl_wq);
			kfree(smd_ctl_devp[i]);
			goto error2;
		}

		smd_ctl_devp[i]->devicep =
			device_create(smd_ctl_classp,
				      NULL,
				      (smd_ctl_number + i),
				      NULL,
				      DEVICE_NAME "%d",
				      i);

		if (IS_ERR(smd_ctl_devp[i]->devicep)) {
			printk(KERN_ERR "%s:%i:%s: "
			       "device_create() ENOMEM\n",
			       __FILE__,
			       __LINE__,
			       __func__);
			r = -ENOMEM;
			cdev_del(&smd_ctl_devp[i]->cdev);
			destroy_workqueue(smd_ctl_devp[i]->ctl_wq);
			kfree(smd_ctl_devp[i]);
			goto error2;
		}

		smd_ctl_devp[i]->nb.notifier_call = modem_notifier;
		modem_register_notifier(&smd_ctl_devp[i]->nb);
		mutex_init(&smd_ctl_devp[i]->has_reset_lock);

	}

	printk(KERN_INFO "SMD Control Port Driver Initialized.\n");
	return 0;

 error2:
	if (i > 0) {
		while (--i >= 0) {
			cdev_del(&smd_ctl_devp[i]->cdev);
			destroy_workqueue(smd_ctl_devp[i]->ctl_wq);
			kfree(smd_ctl_devp[i]);
			device_destroy(smd_ctl_classp,
				       MKDEV(MAJOR(smd_ctl_number), i));
		}
	}

	class_destroy(smd_ctl_classp);
 error1:
	unregister_chrdev_region(MAJOR(smd_ctl_number), NUM_SMD_CTL_PORTS);
 error0:
	return r;
}

static void __exit smd_ctl_cleanup(void)
{
	int i;

	for (i = 0; i < NUM_SMD_CTL_PORTS; ++i) {
		modem_unregister_notifier(&smd_ctl_devp[i]->nb);
		cdev_del(&smd_ctl_devp[i]->cdev);
		kfree(smd_ctl_devp[i]);
		device_destroy(smd_ctl_classp,
			       MKDEV(MAJOR(smd_ctl_number), i));
	}

	class_destroy(smd_ctl_classp);

	unregister_chrdev_region(MAJOR(smd_ctl_number), NUM_SMD_CTL_PORTS);
}

module_init(smd_ctl_init);
module_exit(smd_ctl_cleanup);

MODULE_DESCRIPTION("MSM Shared Memory Control Port");
MODULE_LICENSE("GPL v2");
