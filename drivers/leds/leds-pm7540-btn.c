/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 * Author: Alina Yakovleva <qvdh43@motorola.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/*
 * PM7540 Buttons LED Driver for Morrison
 *
 * Alina Yakovleva qvdh43@motorola.com
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <mach/mpp.h>
#include <mach/vreg.h>
#include <mach/vboost.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "als.h"

#define MODULE_NAME "pm7540_btn_leds"
#define MPP_NAME "mpp7"
//#define PM7540_BTN_LEDS_DEBUG

enum {
    TRACE_SUSPEND = 0x1,
    TRACE_ALS = 0x2,
    TRACE_BRIGHTNESS = 0x4,
};

static unsigned do_trace = TRACE_SUSPEND | TRACE_ALS | TRACE_BRIGHTNESS;
module_param(do_trace, uint, 0644);

static unsigned isink_level = PM_MPP__I_SINK__LEVEL_10mA;
module_param(isink_level, uint, 0644);


#define printk_br(fmt,args...) if (do_trace & TRACE_BRIGHTNESS) printk(KERN_INFO fmt, ##args)
#define printk_suspend(fmt,args...) if (do_trace & TRACE_SUSPEND) printk(KERN_INFO fmt, ##args)
#define printk_als(fmt,args...) if (do_trace & TRACE_ALS) printk(KERN_INFO fmt, ##args)

static DEFINE_MUTEX(pm7540_btn_leds_mutex);

#ifdef PM7540_BTN_LEDS_DEBUG
static int pm7540_btn_leds_open (struct inode *inode, struct file *file);
static int pm7540_btn_leds_release (struct inode *inode, struct file *file);
static ssize_t pm7540_btn_leds_write (struct file *fp, const char __user *buf,
             size_t count, loff_t *pos);
#endif

static void pm7540_btn_do_brightness_set (pm_mpp_i_sink_level_type level);

#ifdef CONFIG_PM
static int pm7540_btn_leds_suspend(struct platform_device *dev, 
    pm_message_t state);
static int pm7540_btn_leds_resume(struct platform_device *dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pm7540_btn_leds_early_suspend (struct early_suspend *h);
static void pm7540_btn_leds_late_resume (struct early_suspend *h);

static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 4, // 1 below LM3535
    .suspend = pm7540_btn_leds_early_suspend,
    .resume = pm7540_btn_leds_late_resume,
};
#endif
#endif

#ifdef PM7540_BTN_LEDS_DEBUG
static atomic_t pm7540_debug;
#endif
static atomic_t suspended;
static atomic_t als_zone;

static als_is_dark_func als_is_dark = lm3535_als_is_dark;

static void pm7540_btn_do_brightness_set (pm_mpp_i_sink_level_type level)
{
    static struct mpp *mpp = NULL;
    static struct vreg *vreg = NULL;
    pm_mpp_i_sink_switch_type stype;
    unsigned config;
    int ret;

    pr_debug ("%s: %d\n", __FUNCTION__, level);
    if (mpp == NULL)
        mpp = mpp_get (NULL, MPP_NAME);
    if (level == PM_MPP__I_SINK__LEVEL_INVALID) {
        level = isink_level;
        stype = PM_MPP__I_SINK__SWITCH_DIS;
        vboost_disable (VBOOST_PM7540_BTN);
    } else {
        stype = PM_MPP__I_SINK__SWITCH_ENA;
        vboost_enable (VBOOST_PM7540_BTN);
        if (vreg == NULL)
            vreg = vreg_get (0, "boost");
        vreg_set_level (vreg, 5000);
    }
    config = (level << 16) | stype;
    ret = mpp_config_analog_sink (mpp, config);
    if (ret) {
        printk (KERN_ERR "%s: mpp_config_analog_sink %s failed: %d\n",
            __FUNCTION__, MPP_NAME, ret);
    }
}

static void pm7540_btn_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value)
{
#ifdef PM7540_BTN_LEDS_DEBUG
    if (atomic_read (&pm7540_debug)) {
        printk (KERN_ERR "%s: %s, %d, in debug mode, ignoring\n", 
            __FUNCTION__, led_cdev->name, value);
        return;
    }
#endif
    mutex_lock (&pm7540_btn_leds_mutex);
    if (!strstr (led_cdev->name, "tcmd")) {
        if (!machine_is_motus ()) {
            if (value && atomic_read (&als_zone)) {
                printk_br ("%s: %s, value=%d, not dark (zone %d), ignore\n",
                    __FUNCTION__, led_cdev->name, value,
                    atomic_read (&als_zone));
                mutex_unlock (&pm7540_btn_leds_mutex);
                return;
            }
        }
    }
    printk_br ("%s: %s, %d=>%d\n", __FUNCTION__, led_cdev->name, 
        led_cdev->brightness, value);
    if (value == -1) {
        value = led_cdev->brightness;
    }
    pm7540_btn_do_brightness_set (value ? isink_level :
        PM_MPP__I_SINK__LEVEL_INVALID);
    mutex_unlock (&pm7540_btn_leds_mutex);
}

static struct led_classdev pm7540_btn_leds = {
    .name                  = "button-backlight",
    .brightness_set        = pm7540_btn_brightness_set,
};

static struct led_classdev pm7540_btn_tcmd_leds = {
    .name                  = "button-tcmd",
    .brightness_set        = pm7540_btn_brightness_set,
};

#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void pm7540_btn_leds_early_suspend (struct early_suspend *h)
{
    pm7540_btn_leds_suspend (NULL, PMSG_SUSPEND);
}

static void pm7540_btn_leds_late_resume (struct early_suspend *h)
{
    pm7540_btn_leds_resume (NULL);
}
#endif

static int pm7540_btn_leds_suspend (struct platform_device *dev, 
    pm_message_t state)
{
#ifdef PM7540_BTN_LEDS_DEBUG
    if (atomic_read (&pm7540_debug)) {
        printk (KERN_ERR "%s: state = %d, in debug mode, ignoring\n", 
            __FUNCTION__, state.event);
        return 0;
    }
#endif

    led_classdev_suspend (&pm7540_btn_leds);
    printk_suspend ("%s: driver suspended, event = %d\n", 
        __FUNCTION__, state.event);
    return 0;
}

static int pm7540_btn_leds_resume (struct platform_device *dev)
{
#ifdef PM7540_BTN_LEDS_DEBUG
    if (atomic_read (&pm7540_debug)) {
        printk (KERN_ERR "%s: in debug mode, ignoring\n", 
            __FUNCTION__);
        return 0;
    }
#endif

    printk_suspend ("%s: resuming, ALS zone is %d, value %d\n", 
        __FUNCTION__, atomic_read (&als_zone),
        pm7540_btn_leds.brightness);
    led_classdev_resume (&pm7540_btn_leds);
    return 0;
}
#endif

#ifdef PM7540_BTN_LEDS_DEBUG
static const struct file_operations pm7540_btn_leds_fops = {
    .owner      = THIS_MODULE,
    .write      = pm7540_btn_leds_write,
    .open       = pm7540_btn_leds_open,
    .release    = pm7540_btn_leds_release,
};

static struct miscdevice pm7540_btn_leds_miscdev = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "pm7540_btnleds_dbg",
    .fops       = &pm7540_btn_leds_fops,
};

static int pm7540_btn_leds_open (struct inode *inode, struct file *file)
{
    return 0;
}

static int pm7540_btn_leds_release (struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t pm7540_btn_leds_write (struct file *fp, const char __user *buf,
             size_t count, loff_t *pos)
{
    unsigned char cmd[10];
    int len = count < 10 ? count : 9;
    unsigned value;
    pm_mpp_i_sink_level_type level;

    if (count < 1)
        return 0;

    if (copy_from_user(cmd, buf, len)) {
        printk (KERN_ERR "%s: unable to copy from user\n", __FUNCTION__);
        return -EFAULT;
    }

    cmd[len] = 0;
    if (cmd[len-1] == '\n') {
        cmd[len-1] = '\0';
        len--;
        if (len == 0)
            return 0;
    }
    printk (KERN_INFO "%s: got command \"%s\" of length %d\n", 
        __FUNCTION__, cmd, len);
    if (!strcmp (cmd, "start")) {
        atomic_set (&pm7540_debug, 1);
        printk (KERN_INFO "%s: enter debug mode\n", __FUNCTION__);
        return strlen (cmd);
    }
    if (!strcmp (cmd, "stop")) {
        atomic_set (&pm7540_debug, 0);
        printk (KERN_INFO "%s: exit debug mode\n", __FUNCTION__);
        return strlen (cmd);
    }
    if (atomic_read (&pm7540_debug)) {
        value = simple_strtoul (cmd, NULL, 10);
        printk (KERN_INFO "%s: set brightness to %dmA\n", __FUNCTION__, value);
        /* Value is in mA, need to convert */
        switch (value) {
            case  (0): level = PM_MPP__I_SINK__LEVEL_INVALID; break; 
            case  (5): level = PM_MPP__I_SINK__LEVEL_5mA; break; 
            case (10): level = PM_MPP__I_SINK__LEVEL_10mA; break; 
            case (15): level = PM_MPP__I_SINK__LEVEL_15mA; break; 
            case (20): level = PM_MPP__I_SINK__LEVEL_20mA; break; 
            case (25): level = PM_MPP__I_SINK__LEVEL_25mA; break; 
            case (30): level = PM_MPP__I_SINK__LEVEL_30mA; break; 
            case (35): level = PM_MPP__I_SINK__LEVEL_35mA; break; 
            case (40): level = PM_MPP__I_SINK__LEVEL_40mA; break; 
            default: level = isink_level; break; 
        }
        pm7540_btn_do_brightness_set (level);
        return strlen (cmd);
    } else {
        printk (KERN_ERR "%s: received unknown command: \"%s\"\n",
            __FUNCTION__, cmd);
        return -EINVAL;
    }
}
#endif

static void pm7540_btn_als_changed (unsigned old_zone, unsigned zone,
    uint32_t cookie)
{
    if (zone >= NUM_ALS_ZONES) {
        printk (KERN_ERR "%s: invalid ALS zone %d\n", __FUNCTION__, zone);
        return;
    }
    atomic_set (&als_zone, zone);
#ifdef CONFIG_DRIVER_ALS_CONTROL
    if (zone > 0) {
        pm7540_btn_brightness_set (&pm7540_btn_leds, 0);
    } else {
        pm7540_btn_brightness_set (&pm7540_btn_leds, -1);
    }
#endif
    printk_als ("%s: ALS zone changed: %d => %d\n",
        __FUNCTION__, old_zone, zone);
}


static int pm7540_btn_leds_probe(struct platform_device *pdev)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    if (machine_is_zeppelin ()) {
        als_is_dark = adp8862_als_is_dark;
    }
    if (machine_is_motus ()) {
        isink_level = PM_MPP__I_SINK__LEVEL_5mA;
    }
    ret = led_classdev_register (&pdev->dev, &pm7540_btn_leds);
    if (ret) {
        printk (KERN_ERR "%s: unable to register LED class %s: error %d\n",
            __FUNCTION__, pm7540_btn_leds.name, ret);
        return ret;
    }
    ret = led_classdev_register (&pdev->dev, &pm7540_btn_tcmd_leds);
    if (ret) {
        printk (KERN_ERR "%s: unable to register LED class %s: error %d\n",
            __FUNCTION__, pm7540_btn_tcmd_leds.name, ret);
    }
#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend (&early_suspend_data);
#endif

#ifdef PM7540_BTN_LEDS_DEBUG
    if ((ret = misc_register (&pm7540_btn_leds_miscdev))) {
        printk (KERN_ERR "%s: misc_register %s failed, error %d\n",
            __FUNCTION__, pm7540_btn_leds_miscdev.name, ret);
    }
#endif
    atomic_set (&suspended, 0);
    if (als_is_dark ()) {
        atomic_set (&als_zone, 0);
    } else {
        atomic_set (&als_zone, 1);
    }
    printk (KERN_INFO "%s: ALS zone is %d\n",
        __FUNCTION__, atomic_read (&als_zone));
    lm3535_register_als_callback (pm7540_btn_als_changed, 0);
    printk (KERN_INFO "%s: registered ALS callback 0x%x\n",
        __FUNCTION__, (unsigned)pm7540_btn_als_changed);

    return ret;
}

static int pm7540_btn_leds_remove(struct platform_device *pdev)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    led_classdev_unregister (&pm7540_btn_leds);
    led_classdev_unregister (&pm7540_btn_tcmd_leds);
#ifdef PM7540_BTN_LEDS_DEBUG
    misc_deregister (&pm7540_btn_leds_miscdev);
#endif

    return 0;
}

static void pm7540_btn_leds_dev_release (struct device *dev)
{
}

static struct platform_device pm7540_btn_leds_device = {
    .name = "pm7540-btn-leds",
    .dev = {
        .release = pm7540_btn_leds_dev_release,
    }
};

static struct platform_driver pm7540_btn_leds_driver = {
    .probe        = pm7540_btn_leds_probe,
    .remove        = pm7540_btn_leds_remove,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = pm7540_btn_leds_suspend,
    .resume        = pm7540_btn_leds_resume,
#endif
#endif
    .driver        = {
        .name        = "pm7540-btn-leds",
        .owner = THIS_MODULE,
    },
};


static int __init pm7540_btn_leds_init(void)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    ret = platform_driver_register(&pm7540_btn_leds_driver);
    if (ret) {
        printk (KERN_ERR "%s: platform_driver_register returned %d\n",
            __FUNCTION__, ret);
    }
    ret = platform_device_register(&pm7540_btn_leds_device);
    if (ret) {
        printk (KERN_ERR "%s: platform_device_register returned %d\n",
            __FUNCTION__, ret);
    }

    return ret;
}

static void __exit pm7540_btn_leds_exit(void)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    platform_driver_unregister(&pm7540_btn_leds_driver);
    platform_device_unregister(&pm7540_btn_leds_device);
}

module_init(pm7540_btn_leds_init);
module_exit(pm7540_btn_leds_exit);

MODULE_AUTHOR("Alina Yakovleva qvdh43@motorola.com");
MODULE_DESCRIPTION("PM7540 Buttons LED driver");
MODULE_LICENSE("GPL");
