/*
 * OMAP Power Management Common Routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include <plat/resource.h>

#include "pm.h"

#ifdef CONFIG_OMAP_PM_SRF
static ssize_t vdd_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &vdd1_opp_attr)
		return sprintf(buf, "%hu\n", resource_get_level("vdd1_opp"));
	else if (attr == &vdd2_opp_attr)
		return sprintf(buf, "%hu\n", resource_get_level("vdd2_opp"));
	else
		return -EINVAL;
}

static ssize_t vdd_opp_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1)
		return -EINVAL;

	if (attr == &vdd1_opp_attr) {
		if (value < 1 || value > 5) {
			printk(KERN_ERR "vdd_opp_store: Invalid value\n");
			return -EINVAL;
		}
		resource_request("vdd1_opp", &dummy_sysfs_dev, value);
	} else if (attr == &vdd2_opp_attr) {
		if (value < 2 || value > 3) {
			printk(KERN_ERR "vdd_opp_store: Invalid value\n");
			return -EINVAL;
		}
		resource_request("vdd2_opp", &dummy_sysfs_dev, value);
	} else {
		return -EINVAL;
	}
	return n;
}
#endif

static int __init omap_pm_init(void)
{
	int error = -EINVAL;

#ifdef CONFIG_OMAP_PM_SRF
	error = sysfs_create_file(power_kobj,
				  &vdd1_opp_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj,
				  &vdd2_opp_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
#endif

	return error;
}
late_initcall(omap_pm_init);
