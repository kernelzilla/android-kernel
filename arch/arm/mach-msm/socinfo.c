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
/*
 * SOC Info Routines
 *
 */

#include <linux/types.h>
#include <linux/sysdev.h>
#include "socinfo.h"
#include "smd_private.h"

/* Used to parse shared memory.  Must match the modem. */
struct socinfo_legacy {
	uint32_t format;
	uint32_t id;
	uint32_t version;
	char baseband_id[32];
};

#include "../../../init/mot_version.h"

const char build_id[32]=PRODUCT_VERSION_STR;

struct socinfo_raw {
	struct socinfo_legacy legacy;

	/* only valid when format==2 */
	uint32_t raw_id;
	uint32_t raw_version;
};

static union {
	struct socinfo_legacy legacy;
	struct socinfo_raw raw;
} *socinfo;

static enum msm_cpu cpu_of_id[] = {
	/* Uninitialized IDs are not known to run Linux.
	 * MSM_CPU_UNKNOWN is set to 0 to ensure these IDs are
	 * considered as unknown CPUs. */

	/* 7x01 IDs */
	[1]  = MSM_CPU_7X01,
	[16] = MSM_CPU_7X01,
	[17] = MSM_CPU_7X01,
	[18] = MSM_CPU_7X01,
	[19] = MSM_CPU_7X01,
	[23] = MSM_CPU_7X01,
	[25] = MSM_CPU_7X01,
	[26] = MSM_CPU_7X01,
	[32] = MSM_CPU_7X01,
	[33] = MSM_CPU_7X01,
	[34] = MSM_CPU_7X01,
	[35] = MSM_CPU_7X01,

	/* 7x25 IDs */
	[20] = MSM_CPU_7X25,
	[21] = MSM_CPU_7X25,
	[24] = MSM_CPU_7X25,
	[27] = MSM_CPU_7X25,
	[39] = MSM_CPU_7X25,
	[40] = MSM_CPU_7X25,
	[41] = MSM_CPU_7X25,
	[42] = MSM_CPU_7X25,

	/* 7x27 IDs */
	[43] = MSM_CPU_7X27,
	[44] = MSM_CPU_7X27,

	/* 8x50 IDs */
	[30] = MSM_CPU_8X50,
	[36] = MSM_CPU_8X50,
	[37] = MSM_CPU_8X50,
	[38] = MSM_CPU_8X50,

	/* Last known ID. */
	[53] = MSM_CPU_UNKNOWN,
};

static enum msm_cpu cur_cpu;

uint32_t socinfo_get_id(void)
{
	return (socinfo) ? socinfo->legacy.id : 0;
}

uint32_t socinfo_get_version(void)
{
	return (socinfo) ? socinfo->legacy.version : 0;
}

char *socinfo_get_baseband_id(void)
{
	return (socinfo) ? socinfo->legacy.baseband_id : NULL;
}
EXPORT_SYMBOL (socinfo_get_baseband_id);

char *socinfo_get_build_id(void)
{
	return &build_id[0];
}

uint32_t socinfo_get_raw_id(void)
{
	return socinfo ?
		(socinfo->legacy.format == 2 ? socinfo->raw.raw_id : 0)
		: 0;
}

uint32_t socinfo_get_raw_version(void)
{
	return socinfo ?
		(socinfo->legacy.format == 2 ? socinfo->raw.raw_version : 0)
		: 0;
}

enum msm_cpu socinfo_get_msm_cpu(void)
{
	return cur_cpu;
}

static ssize_t
socinfo_show_id(struct sys_device *dev,
		struct sysdev_attribute *attr,
		char *buf)
{
	if (!socinfo) {
		printk(KERN_ERR "%s: No socinfo found!", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", socinfo_get_id());
}

static ssize_t
socinfo_show_version(struct sys_device *dev,
		     struct sysdev_attribute *attr,
		     char *buf)
{
	uint32_t version;

	if (!socinfo) {
		printk(KERN_ERR "%s: No socinfo found!", __func__);
		return 0;
	}

	version = socinfo_get_version();
	return snprintf(buf, PAGE_SIZE, "%u.%u\n",
			SOCINFO_VERSION_MAJOR(version),
			SOCINFO_VERSION_MINOR(version));
}

static ssize_t
socinfo_show_baseband_id(struct sys_device *dev,
		      struct sysdev_attribute *attr,
		      char *buf)
{
	if (!socinfo) {
		printk(KERN_ERR "%s: No socinfo found!", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%-.32s\n", socinfo_get_baseband_id());
}

static ssize_t
socinfo_show_build_id(struct sys_device *dev,
		      struct sysdev_attribute *attr,
		      char *buf)
{
	if (!socinfo) {
		printk(KERN_ERR "%s: No socinfo found!", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%-.32s\n", socinfo_get_build_id());
}

static ssize_t
socinfo_show_raw_id(struct sys_device *dev,
		    struct sysdev_attribute *attr,
		    char *buf)
{
	if (!socinfo) {
		printk(KERN_ERR "%s: No socinfo found!", __func__);
		return 0;
	}
	if (socinfo->legacy.format != 2) {
		printk(KERN_ERR "%s: Raw ID not available!", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", socinfo_get_raw_id());
}

static ssize_t
socinfo_show_raw_version(struct sys_device *dev,
			 struct sysdev_attribute *attr,
			 char *buf)
{
	if (!socinfo) {
		printk(KERN_ERR "%s: No socinfo found!", __func__);
		return 0;
	}
	if (socinfo->legacy.format != 2) {
		printk(KERN_ERR "%s: Raw version not available!", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", socinfo_get_raw_version());
}

static struct sysdev_attribute socinfo_legacy_files[] = {
	_SYSDEV_ATTR(id, 0444, socinfo_show_id, NULL),
	_SYSDEV_ATTR(version, 0444, socinfo_show_version, NULL),
	_SYSDEV_ATTR(baseband_id, 0444, socinfo_show_baseband_id, NULL),
	_SYSDEV_ATTR(build_id, 0444, socinfo_show_build_id, NULL),
};

static struct sysdev_attribute socinfo_raw_files[] = {
	_SYSDEV_ATTR(raw_id, 0444, socinfo_show_raw_id, NULL),
	_SYSDEV_ATTR(raw_version, 0444, socinfo_show_raw_version, NULL),
};

static struct sysdev_class soc_sysdev_class = {
	.name = "soc",
};

static struct sys_device soc_sys_device = {
	.id = 0,
	.cls = &soc_sysdev_class,
};

static void __init socinfo_create_files(struct sys_device *dev,
					struct sysdev_attribute files[],
					int size)
{
	int i;
	for (i = 0; i < size; i++) {
		int err = sysdev_create_file(dev, &files[i]);
		if (err) {
			printk(KERN_ERR "%s: sysdev_create_file(%s)=%d\n",
			       __func__, files[i].attr.name, err);
			return;
		}
	}
}

static void __init socinfo_init_sysdev(void)
{
	int err;

	err = sysdev_class_register(&soc_sysdev_class);
	if (err) {
		printk(KERN_ERR "%s: sysdev_class_register fail (%d)\n",
		       __func__, err);
		return;
	}
	err = sysdev_register(&soc_sys_device);
	if (err) {
		printk(KERN_ERR "%s: sysdev_register fail (%d)\n",
		       __func__, err);
		return;
	}
	socinfo_create_files(&soc_sys_device, socinfo_legacy_files,
				ARRAY_SIZE(socinfo_legacy_files));
	if (socinfo->legacy.format != 2)
		return;
	socinfo_create_files(&soc_sys_device, socinfo_raw_files,
				ARRAY_SIZE(socinfo_raw_files));
}

int __init socinfo_init(void)
{
	socinfo = smem_alloc(SMEM_HW_SW_BUILD_ID, sizeof(struct socinfo_raw));
	if (!socinfo)
		socinfo = smem_alloc(SMEM_HW_SW_BUILD_ID,
				sizeof(struct socinfo_legacy));

	if (!socinfo) {
		printk(KERN_ERR "%s: Can't find SMEM_HW_SW_BUILD_ID\n",
		       __func__);
		return -EIO;
	}

	WARN(!socinfo_get_id(), "Unknown SOC ID!\n");
	WARN(socinfo_get_id() >= ARRAY_SIZE(cpu_of_id),
		"New IDs added! ID => CPU mapping might need an update.\n");

	if (socinfo->legacy.id < ARRAY_SIZE(cpu_of_id))
		cur_cpu = cpu_of_id[socinfo->legacy.id];

	socinfo_init_sysdev();
	return 0;
}
