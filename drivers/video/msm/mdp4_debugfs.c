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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <linux/debugfs.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"


#define MDP4_DEBUG_BUF	128


static char mdp4_debug_buf[MDP4_DEBUG_BUF];
static ulong mdp4_debug_offset;
static ulong mdp4_base_addr;

static int mdp4_offset_set(void *data, u64 val)
{
	mdp4_debug_offset = (int)val;
	return 0;
}

static int mdp4_offset_get(void *data, u64 *val)
{
	*val = (u64)mdp4_debug_offset;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(
			mdp4_offset_fops,
			mdp4_offset_get,
			mdp4_offset_set,
			"%llx\n");


static int mdp4_debugfs_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mdp4_debugfs_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mdp4_debugfs_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int cnt;
	unsigned int data;

	printk(KERN_INFO "%s: offset=%d count=%d *ppos=%d\n",
		__func__, (int)mdp4_debug_offset, (int)count, (int)*ppos);

	if (count > sizeof(mdp4_debug_buf))
		return -EFAULT;

	if (copy_from_user(mdp4_debug_buf, buff, count))
		return -EFAULT;


	mdp4_debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(mdp4_debug_buf, "%x", &data);
	if (cnt < 1) {
		printk(KERN_ERR "%s: sscanf failed cnt=%d" , __func__, cnt);
		return -EINVAL;
	}

	writel(&data, mdp4_base_addr + mdp4_debug_offset);

	return 0;
}

static ssize_t mdp4_debugfs_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	unsigned int data;

	printk(KERN_INFO "%s: offset=%d count=%d *ppos=%d\n",
		__func__, (int)mdp4_debug_offset, (int)count, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	data = readl(mdp4_base_addr + mdp4_debug_offset);

	len = snprintf(mdp4_debug_buf, 4, "%x\n", data);

	if (len > 0) {
		if (len > count)
			len = count;
		if (copy_to_user(buff, mdp4_debug_buf, len))
			return -EFAULT;
	}

	printk(KERN_INFO "%s: len=%d\n", __func__, len);

	if (len < 0)
		return 0;

	*ppos += len;	/* increase offset */

	return len;
}

static const struct file_operations mdp4_debugfs_fops = {
	.open = mdp4_debugfs_open,
	.release = mdp4_debugfs_release,
	.read = mdp4_debugfs_read,
	.write = mdp4_debugfs_write,
};

int mdp4_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("mdp4", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("offset", 0644, dent, 0, &mdp4_offset_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: offset fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("regs", 0644, dent, 0, &mdp4_debugfs_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: regs fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	mdp4_debug_offset = 0;
	mdp4_base_addr = (ulong) msm_mdp_base;	/* defined at msm_fb_def.h */

	return 0;
}
