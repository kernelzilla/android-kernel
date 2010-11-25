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

/* Qualcomm PMIC Multi-Purpose Pin Configurations */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/debugfs.h>

#include <mach/mpp.h>

#include "proc_comm.h"

#define MPP(_name, _id, _status) { .name = _name, .id = _id, .status = _status}

static struct mpp mpps[] = {
	MPP("mpp1", 0, 0),
	MPP("mpp2", 1, 0),
	MPP("mpp3", 2, 0),
	MPP("mpp4", 3, 0),
	MPP("mpp5", 4, 0),
	MPP("mpp6", 5, 0),
	MPP("mpp7", 6, 0),
	MPP("mpp8", 7, 0),
	MPP("mpp9", 8, 0),
	MPP("mpp10", 9, 0),
	MPP("mpp11", 10, 0),
	MPP("mpp12", 11, 0),
	MPP("mpp13", 12, 0),
	MPP("mpp14", 13, 0),
	MPP("mpp15", 14, 0),
	MPP("mpp16", 15, 0),
	MPP("mpp17", 16, 0),
	MPP("mpp18", 17, 0),
	MPP("mpp19", 18, 0),
	MPP("mpp20", 19, 0),
	MPP("mpp21", 20, 0),
	MPP("mpp22", 21, 0),
};

struct mpp *mpp_get(struct device *dev, const char *id)
{
	int n;
	for (n = 0; n < ARRAY_SIZE(mpps); n++) {
		if (!strcmp(mpps[n].name, id))
			return mpps + n;
	}
	return NULL;
}
EXPORT_SYMBOL(mpp_get);

int mpp_config_digital_out(struct mpp *mpp, unsigned config)
{
	unsigned id = mpp->id;
	int err;
	err = msm_proc_comm(PCOM_PM_MPP_CONFIG, &id, &config);
	mpp->status = err;
	return err;
}
EXPORT_SYMBOL(mpp_config_digital_out);

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_LAJOLLA)
int mpp_config_analog_sink(struct mpp *mpp, unsigned config)
{
	unsigned id = mpp->id;
	int err;
	err = msm_proc_comm(PCOM_PM_MPP_CONFIG_I_SINK, &id, &config);
	mpp->status = err;
	return err;
}
EXPORT_SYMBOL(mpp_config_analog_sink);
#endif


#if defined(CONFIG_DEBUG_FS)
static int mpp_debug_set(void *data, u64 val)
{
	int err;
	struct mpp *mpp = data;

	err = mpp_config_digital_out(mpp, (unsigned)val);
	if (err) {
		printk(KERN_ERR
			   "%s: mpp_config_digital_out \
			   [%s(%d) = 0x%x] failed\n",
			   __func__, mpp->name, mpp->id, (unsigned)val);
	}
	return 0;
}

static int mpp_debug_get(void *data, u64 *val)
{
	struct mpp *mpp = data;
	int status = mpp->status;
	if (!status)
		*val = 0;
	else
		*val = 1;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mpp_fops, mpp_debug_get, mpp_debug_set, "%llu\n");

static int __init mpp_debug_init(void)
{
	struct dentry *dent;
	int n;

	dent = debugfs_create_dir("mpp", 0);
	if (IS_ERR(dent))
		return 0;

	for (n = 0; n < ARRAY_SIZE(mpps); n++)
		debugfs_create_file(mpps[n].name, 0644, dent, mpps + n,
				    &mpp_fops);

	return 0;
}

device_initcall(mpp_debug_init);
#endif
