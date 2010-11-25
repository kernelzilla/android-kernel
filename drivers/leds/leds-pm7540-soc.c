/*
 * Copyright (C) 2008-2009 Motorola, Inc.
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
 * PM7540 SOC LED Driver for Morrison
 *
 * Alina Yakovleva qvdh43@motorola.com
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <mach/mpp.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define MODULE_NAME "pm7540_soc_leds"
#define DEFAULT_ISINK_LEVEL PM_MPP__I_SINK__LEVEL_5mA
//#define PM7540_SOC_LEDS_DEBUG
extern void msm_blink_charging_led (uint16_t onoff, uint16_t level,
    uint16_t on_ms, uint16_t off_ms);
static char *isink_level_toname (unsigned level);

struct pm7540_soc_data {
    unsigned brightness;
    unsigned blink_on;
    unsigned blink_off;
    unsigned default_level;
};

static DEFINE_MUTEX(mutex);

static struct pm7540_soc_data this_data = {
    .default_level = DEFAULT_ISINK_LEVEL,
};
module_param_named (isink_level, this_data.default_level, uint, 0664);

static void pm7540_soc_brightness_set (struct led_classdev *led_cdev,
    enum led_brightness value)
{
    printk (KERN_INFO "%s: %s, %d\n", __FUNCTION__, led_cdev->name, value);
    mutex_lock (&mutex);
    this_data.brightness = value;
    msm_blink_charging_led (value ? 1 : 0, this_data.default_level, 0, 0);
    mutex_unlock (&mutex);
}

static struct led_classdev this_led = {
    .name                  = "messaging",
    .brightness_set        = pm7540_soc_brightness_set,
};

static ssize_t pm7540_soc_blink_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    mutex_lock (&mutex);
    sprintf (buf, "%d, on=%dms off=%dms, default level=%s (%d)\n",
        this_data.brightness,
        this_data.blink_on, this_data.blink_off,
        isink_level_toname (this_data.default_level),
        this_data.default_level);
    mutex_unlock (&mutex);
    return strlen(buf) + 1;
}

static ssize_t pm7540_soc_blink_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    int n;
    struct led_classdev *led_cdev;
    unsigned msOn = 0, msOff = 0, bvalue = 0;
    unsigned level = this_data.default_level;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    led_cdev = dev_get_drvdata (dev);

    /* The format is: "brightness msOn msOff" */
    n = sscanf (buf, "%d %d %d %d", &bvalue, &msOn, &msOff, &level);

    if (n < 3) {
        printk (KERN_ERR "%s: invalid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }

    if (level == 0) {
        level = this_data.default_level;
    }
    printk (KERN_INFO "%s: %s, b = %d, msOn = %d, msOff = %d, level = %s (%d)\n",
        __FUNCTION__, led_cdev->name, bvalue, msOn, msOff,
        isink_level_toname (level), level);
    mutex_lock (&mutex);
    msm_blink_charging_led (bvalue ? 1 : 0, level, msOn, msOff);
    this_data.brightness = bvalue;
    this_data.blink_on = msOn;
    this_data.blink_off = msOff;
    mutex_unlock (&mutex);
    return size;
}

static DEVICE_ATTR(blink, 0666, pm7540_soc_blink_show, pm7540_soc_blink_store);

char *isink_level_toname (unsigned level)
{
    switch (level) {
        case  (PM_MPP__I_SINK__LEVEL_5mA): return "5mA"; break;
        case (PM_MPP__I_SINK__LEVEL_10mA): return "10mA"; break;
        case (PM_MPP__I_SINK__LEVEL_15mA): return "15mA"; break;
        case (PM_MPP__I_SINK__LEVEL_20mA): return "20mA"; break;
        case (PM_MPP__I_SINK__LEVEL_25mA): return "25mA"; break;
        case (PM_MPP__I_SINK__LEVEL_30mA): return "30mA"; break;
        case (PM_MPP__I_SINK__LEVEL_35mA): return "35mA"; break;
        case (PM_MPP__I_SINK__LEVEL_40mA): return "40mA"; break;
        case (PM_MPP__I_SINK__LEVEL_INVALID): return "INVALID"; break;
        default: return "UNKNOWN"; break;
    }
    return "UNKNOWN";
}

#ifdef PM7540_SOC_LEDS_DEBUG
static ssize_t pm7540_soc_debug_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    sprintf (buf, "isink current = %s\n",
        isink_level_toname (this_data.default_level));
    return strlen (buf) + 1;
}

static ssize_t pm7540_soc_debug_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    struct led_classdev *led_cdev;
    unsigned value;
    char *ptr;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    led_cdev = dev_get_drvdata (dev);

    if (strstr (buf, "current") || strstr (buf, "level")) {
        ptr = strpbrk (buf, "0123456789");
        if (ptr) {
            value = simple_strtoul (ptr, NULL, 10);
            switch (value) {
                case  (5):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_5mA; break;
                case (10):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_10mA; break;
                case (15):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_15mA; break;
                case (20):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_20mA; break;
                case (25):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_25mA; break;
                case (30):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_30mA; break;
                case (35):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_35mA; break;
                case (40):
                  this_data.default_level = PM_MPP__I_SINK__LEVEL_40mA; break;
                default:
                  this_data.default_level = DEFAULT_ISINK_LEVEL; break;
            }
        } else {
            this_data.default_level = DEFAULT_ISINK_LEVEL;
        }
        printk (KERN_INFO "%s: changing default current level to: %s\n",
            __FUNCTION__, isink_level_toname (this_data.default_level));
        return size;
    }
    printk (KERN_ERR "%s: invalid command: %s\n", __FUNCTION__, buf);
    return -EINVAL;
}

static DEVICE_ATTR(debug, 0666, pm7540_soc_debug_show, pm7540_soc_debug_store);
#endif

static int pm7540_soc_leds_probe(struct platform_device *pdev)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
#ifdef CONFIG_MACH_MOT
	if (machine_is_morrison()) {
		printk(KERN_ERR "%s:morrison; returning\n", __FUNCTION__);
		return -EINVAL;
	}
#endif
#if 0
	if (!machine_is_motus()) {
		printk(KERN_ERR "%s: not motus; returning\n", __FUNCTION__);
		return -EINVAL;
	}
#endif
    ret = led_classdev_register (&pdev->dev, &this_led);
    if (ret) {
        printk (KERN_ERR "%s: unable to register LED class %s: error %d\n",
            __FUNCTION__, this_led.name, ret);
        return ret;
    }
    ret = device_create_file (this_led.dev, &dev_attr_blink);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"blink\" device file for %s: %d\n",
            __FUNCTION__, this_led.name, ret);
        led_classdev_unregister (&this_led);
        return ret;
    }
    dev_set_drvdata (this_led.dev, &this_led);

#ifdef PM7540_SOC_LEDS_DEBUG
    ret = device_create_file (this_led.dev, &dev_attr_debug);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"debug\" device file for %s: %d\n",
            __FUNCTION__, this_led.name, ret);
    }
#endif

    return ret;
}

static int pm7540_soc_leds_remove(struct platform_device *pdev)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    led_classdev_unregister (&this_led);
    device_remove_file (this_led.dev, &dev_attr_blink);
#ifdef PM7540_SOC_LEDS_DEBUG
    device_remove_file (this_led.dev, &dev_attr_debug);
#endif

    return 0;
}

static struct platform_device pm7540_soc_leds_device = {
    .name = "pm7540-soc-leds",
};

static struct platform_driver pm7540_soc_leds_driver = {
    .probe        = pm7540_soc_leds_probe,
    .remove        = pm7540_soc_leds_remove,
    .driver        = {
        .name        = "pm7540-soc-leds",
        .owner = THIS_MODULE,
    },
};


static int __init pm7540_soc_leds_init(void)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    ret = platform_driver_register(&pm7540_soc_leds_driver);
    if (ret) {
        printk (KERN_ERR "%s: platform_driver_register returned %d\n",
            __FUNCTION__, ret);
    }
    ret = platform_device_register(&pm7540_soc_leds_device);
    if (ret) {
        printk (KERN_ERR "%s: platform_device_register returned %d\n",
            __FUNCTION__, ret);
    }

    return ret;
}

static void __exit pm7540_soc_leds_exit(void)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    platform_driver_unregister(&pm7540_soc_leds_driver);
    platform_device_unregister(&pm7540_soc_leds_device);
}

module_init(pm7540_soc_leds_init);
module_exit(pm7540_soc_leds_exit);

MODULE_AUTHOR("Alina Yakovleva qvdh43@motorola.com");
MODULE_DESCRIPTION("PM7540 SOC LED driver");
MODULE_LICENSE("GPL");
