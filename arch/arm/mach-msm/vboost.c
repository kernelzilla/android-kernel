/* linux/arch/arm/mach-msm/vboost.c
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 * Author: Alina Yakovleva <qvdh43@motorola.com>
 * 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <mach/vreg.h>
#include <mach/vboost.h>

#include "proc_comm.h"

static DEFINE_MUTEX(mutex);

static uint32_t vboost_usage = 0;
static struct vreg *boost = NULL;

int vboost_enable (uint32_t app_mask)
{
    int ret;

    mutex_lock (&mutex);
    if (boost == NULL) {
        boost = vreg_get (0, "boost");
    }
    ret = vreg_enable (boost);
    if (ret == 0) {
        vboost_usage |= app_mask;
        //printk (KERN_INFO "%s: enabled vboost for 0x%x; umask=0x%x\n",
        //    __FUNCTION__, app_mask, vboost_usage);
    } else {
        printk (KERN_ERR "%s: unable to enable boost for 0x%x: %d\n",
            __FUNCTION__, app_mask, ret);
    }
    mutex_unlock (&mutex);
    return ret;
}
EXPORT_SYMBOL(vboost_enable);

int vboost_disable (uint32_t app_mask)
{
    int ret;

    mutex_lock (&mutex);
    if ((vboost_usage  & (~app_mask)) != 0) {
        // Some other app still need it
        vboost_usage &= ~app_mask;
        //printk (KERN_INFO "%s: will not disable vboost: umask=0x%x\n",
        //    __FUNCTION__, vboost_usage);
        mutex_unlock (&mutex);
        return 0;
    }
    if (boost == NULL) {
        boost = vreg_get (0, "boost");
    }
    ret = vreg_disable (boost);
    if (ret == 0) {
        vboost_usage &= ~app_mask;
        //printk (KERN_INFO "%s: disabled vboost for 0x%x; umask=0x%x\n",
        //    __FUNCTION__, app_mask, vboost_usage);
    } else {
        printk (KERN_ERR "%s: unable to disable boost for 0x%x: %d\n",
            __FUNCTION__, app_mask, ret);
    }
    mutex_unlock (&mutex);
    return ret;
}
EXPORT_SYMBOL(vboost_disable);
