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
 * Morrison Keypad LED Driver
 *
 * Alina Yakovleva qvdh43@motorola.com
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <asm/io.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "als.h"

//#define MORRISON_KPD_DEBUG
#ifdef MORRISON_KPD_DEBUG
static atomic_t morrison_kpd_debug;
#endif

enum {
    TRACE_SUSPEND = 0x1,
    TRACE_ALS = 0x2,
    TRACE_BRIGHTNESS = 0x4,
};

static unsigned do_trace = TRACE_SUSPEND;
module_param(do_trace, uint, 0644);

#define printk_br(fmt,args...) if (do_trace & TRACE_BRIGHTNESS) printk(KERN_INFO fmt, ##args)
#define printk_suspend(fmt,args...) if (do_trace & TRACE_SUSPEND) printk(KERN_INFO fmt, ##args)
#define printk_als(fmt,args...) if (do_trace & TRACE_ALS) printk(KERN_INFO fmt, ##args)

#define MODULE_NAME "leds_morrison_kpd"

extern int adp5588_set_backlight (uint8_t mask);
extern int adp5588_get_backlight (void);
extern void msm_config_gp_mn (uint32_t freq, uint32_t duty);

static void keypad_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value);
static void keypad_seg_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value);
#ifdef CONFIG_PM
static int keypad_leds_suspend (struct platform_device *dev, 
    pm_message_t state);
static int keypad_leds_resume (struct platform_device *dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void keypad_leds_early_suspend (struct early_suspend *h);
static void keypad_leds_late_resume (struct early_suspend *h);

static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = keypad_leds_early_suspend,
    .resume = keypad_leds_late_resume,
};
#endif
#endif

struct kpd_leds_data {
    atomic_t suspended;
    atomic_t als_zone;
    uint32_t default_freq;
    uint32_t default_duty;
    uint8_t segment_mask; // Which segments are currently on
    struct device *dev;
};

static DEFINE_MUTEX (kpd_leds_mutex);

static struct kpd_leds_data data = {
    .default_freq = 10,
    .default_duty = 100,
};

static als_is_dark_func als_is_dark = lm3535_als_is_dark;

static struct led_classdev keypad_leds[] = {
    {
        .name			= "keyboard-backlight",
        .brightness_set		= keypad_brightness_set,
    },
    {
        .name			= "keyboard1-backlight",
        .brightness_set		= keypad_seg_brightness_set,
    },
    {
        .name			= "keyboard2-backlight",
        .brightness_set		= keypad_seg_brightness_set,
    },
    {
        .name			= "keyboard-tcmd",
        .brightness_set		= keypad_brightness_set,
    },
    {
        .name			= "keyboard1-tcmd",
        .brightness_set		= keypad_seg_brightness_set,
    },
    {
        .name			= "keyboard2-tcmd",
        .brightness_set		= keypad_seg_brightness_set,
    }
};

static void keypad_als_changed (unsigned old_zone, unsigned zone, 
    uint32_t cookie)
{
    if (zone >= NUM_ALS_ZONES) {
        printk (KERN_ERR "%s: invalid ALS zone %d\n", __FUNCTION__, zone);
        return;
    }
    atomic_set (&data.als_zone, zone);
    printk_als (KERN_INFO "%s: ALS zone changed: %d => %d\n",
        __FUNCTION__, old_zone, zone);
}

static void keypad_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value)
{
    uint8_t mask = 0x3;

#ifdef MORRISON_KPD_DEBUG
    if (atomic_read (&morrison_kpd_debug)) {
        printk (KERN_ERR "%s: %d, in debug mode, ignoring\n", 
            __FUNCTION__, value);
        return;
    }
#endif

    /* Check ALS zone if this is not a test command */
    if (!strstr (led_cdev->name, "tcmd")) {
        /* Check ALS zone if this is not a test command */
        if (value && atomic_read (&data.als_zone)) {
            printk_br ("%s: %s, value=%d, not dark (zone %d), ignore\n",
                __FUNCTION__, led_cdev->name, value, 
                atomic_read (&data.als_zone));
            return;
        }
    }
    printk_br ("%s: %s, value=%d\n", __FUNCTION__, led_cdev->name, value);
    mutex_lock (&kpd_leds_mutex);
    if (value == 0) {
        /* If this is not resume clear kpd1 and kpd2 */
        if (!atomic_read (&data.suspended)) {
            keypad_leds[1].brightness = 0;
            keypad_leds[2].brightness = 0;
        }
        mask = 0;
    }
    data.segment_mask = mask;
    adp5588_set_backlight (mask);
    if (value == 0) {
        //msleep_interruptible (2000);
    }
    msm_config_gp_mn (data.default_freq, value ? data.default_duty : 0);
    mutex_unlock (&kpd_leds_mutex);
}

static void keypad_seg_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value)
{
    uint8_t mask = 0x1, old_mask;

#ifdef MORRISON_KPD_DEBUG
    if (atomic_read (&morrison_kpd_debug)) {
        printk (KERN_ERR "%s: %d, in debug mode, ignoring\n", 
            __FUNCTION__, value);
        return;
    }
#endif

    /* Check ALS zone if this is not a test command */
    if (!strstr (led_cdev->name, "tcmd")) {
        /* Check ALS zone if this is not a test command */
        if (value && atomic_read (&data.als_zone)) {
            printk_br ("%s: %s, value=%d, not dark (zone %d), ignore\n",
                __FUNCTION__, led_cdev->name, value,
                atomic_read (&data.als_zone));
            return;
        }
    }
    if (strstr (led_cdev->name, "2")) {
        mask = 0x2;
    } 
    mutex_lock (&kpd_leds_mutex);
    old_mask = data.segment_mask;
    if (value) {
        data.segment_mask |= mask;
    } else {
        data.segment_mask &= ~mask;
    }
    printk_br ("%s: %s: %d, mask %d=>%d\n", 
        __FUNCTION__, led_cdev->name, value, old_mask, data.segment_mask);
    adp5588_set_backlight (data.segment_mask);
    if ((data.segment_mask == 0 && old_mask != 0) ||
        (data.segment_mask != 0 && old_mask == 0)) {
#if 0
        if (value == 0) {
            msleep_interruptible (2000);
        }
#endif
        msm_config_gp_mn (data.default_freq, value ? data.default_duty : 0);
    }
    mutex_unlock (&kpd_leds_mutex);
}

static void morrison_kpd_unregister_leds (void)
{
    int i;
    int num_leds = sizeof (keypad_leds) / sizeof (keypad_leds[0]);

    for (i = 0; i < num_leds; i++) {
        led_classdev_unregister (&keypad_leds[i]);
    }
}

static int morrison_kpd_register_leds (struct device *dev)
{
    int i, ret;
    int num_leds = sizeof (keypad_leds) / sizeof (keypad_leds[0]);

    for (i = 0; i < num_leds; i++) {
        ret = led_classdev_register (dev, &keypad_leds[i]);
        if (ret) {
            printk (KERN_ERR "%s: unable to register %s LED: %d\n",
                __FUNCTION__, keypad_leds[i].name, ret);
            // morrison_kpd_unregister_leds (); -- this causes kernel crash/ambulance mode
            return ret;
        }
    }
    return 0;
}
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void keypad_leds_early_suspend (struct early_suspend *h)
{
    keypad_leds_suspend (NULL, PMSG_SUSPEND);
}

static void keypad_leds_late_resume (struct early_suspend *h)
{
    keypad_leds_resume (NULL);
}
#endif

static int keypad_leds_suspend (struct platform_device *dev, 
    pm_message_t state)
{
#ifdef MORRISON_KPD_DEBUG
    if (atomic_read (&morrison_kpd_debug)) {
        printk (KERN_ERR "%s: in debug mode, ignoring\n", __FUNCTION__);
        return 0;
    }
#endif
    if (atomic_read (&data.suspended)) {
        printk (KERN_ERR "%s: is already suspended by PM\n",
            __FUNCTION__);
    }
    //printk (KERN_ERR "%s: enter\n", __FUNCTION__);
	led_classdev_suspend (&keypad_leds[0]);
	led_classdev_suspend (&keypad_leds[1]);
	led_classdev_suspend (&keypad_leds[2]);
    atomic_set (&data.suspended, 1);
    printk_suspend ("%s: driver suspended, event = %d\n", 
        __FUNCTION__, state.event);
	return 0;
}

static int keypad_leds_resume (struct platform_device *dev)
{
#ifdef MORRISON_KPD_DEBUG
    if (atomic_read (&morrison_kpd_debug)) {
        printk (KERN_ERR "%s: in debug mode, ignoring\n", __FUNCTION__);
        return 0;
    }
#endif
    printk_suspend ("%s: resuming, ALS zone is %d, values %d %d %d\n", 
        __FUNCTION__, atomic_read (&data.als_zone),
        keypad_leds[0].brightness, keypad_leds[1].brightness,
        keypad_leds[2].brightness);
    /* If either kpd1 or kpd2 have non-zero value resume then last */
    if (keypad_leds[1].brightness || keypad_leds[2].brightness) {
        /* Simulate resume of keyboard-backlight */
        keypad_leds[0].flags &= ~LED_SUSPENDED;
        //led_classdev_resume (&keypad_leds[0]);
        led_classdev_resume (&keypad_leds[1]);
        led_classdev_resume (&keypad_leds[2]);
    } else {
        led_classdev_resume (&keypad_leds[2]);
        led_classdev_resume (&keypad_leds[1]);
        led_classdev_resume (&keypad_leds[0]);
    }
    atomic_set (&data.suspended, 0);
    printk_suspend ("%s: driver resumed\n", __FUNCTION__);
	return 0;
}
#endif

#ifdef MORRISON_KPD_DEBUG
static ssize_t morrison_kpd_pwm_show (struct device *dev,
				      struct device_attribute *attr, char *buf)
{
    sprintf (buf, "Usage: (start | stop | <duty> [<freq>])\n");
    return strlen(buf) + 1;
}

static ssize_t morrison_kpd_pwm_store (struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
    int freq = 10, duty;
    int n;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    if (strstr (buf, "start")) {
        atomic_set (&morrison_kpd_debug, 1);
        printk (KERN_INFO "%s: enter debug mode\n", __FUNCTION__);
        return size;
    }
    if (strstr (buf, "stop")) {
        atomic_set (&morrison_kpd_debug, 0);
        printk (KERN_INFO "%s: exit debug mode\n", __FUNCTION__);
        return size;
    }
    /* The format is: "duty freq [region]" */
    n = sscanf (buf, "%d %d", &duty, &freq);
    if (n < 1) {
        printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    if (duty < 0 || duty > 100) {
        printk (KERN_ERR "%s: inavlid duty cycle %d%%, should be 0-100\n", 
            __FUNCTION__, freq);
        return -EINVAL;
    }
    if (n > 1) {
        if (freq < 10 || freq > 100) {
            printk (KERN_ERR "%s: inavlid frequency %d kHz, should be 10-100 kHz\n",
                __FUNCTION__, freq);
            return -EINVAL;
        }
    } else {
        freq = data.default_freq;
    }
    if (!atomic_read (&morrison_kpd_debug)) {
        printk (KERN_INFO "%s: changing default freq to %d, duty to %d\n",
            __FUNCTION__, freq, duty);
        data.default_freq = freq;
        data.default_duty = duty;
        return size;
    }
    printk (KERN_INFO "%s: duty=%d, freq=%d\n",
        __FUNCTION__, duty, freq);
    msm_config_gp_mn (freq, duty);
    return size;
}

static DEVICE_ATTR(pwm, 0777, morrison_kpd_pwm_show, morrison_kpd_pwm_store);

static ssize_t morrison_kpd_region_show (struct device *dev,
				      struct device_attribute *attr, char *buf)
{
    int mask;

    mask = adp5588_get_backlight ();
    sprintf (buf, "%d\n", mask);
    return strlen(buf) + 1;
}

static ssize_t morrison_kpd_region_store (struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
    int region = 0;
    int n;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    if (!atomic_read (&morrison_kpd_debug)) {
        printk (KERN_ERR "%s: can't change region: not in debug mode\n", 
            __FUNCTION__);
        return -EINVAL;
    }
    /* The format is: "duty freq [region]" */
    n = sscanf (buf, "%d", &region);
    if (n != 1) {
        printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    if (region < 0 || region > 3) {
        printk (KERN_ERR "%s: inavlid region %d, should be 0 to 3\n",
            __FUNCTION__, region);
        return -EINVAL;
    }
    printk (KERN_INFO "%s: region=%d\n", __FUNCTION__, region);
    adp5588_set_backlight (region);
    return size;
}
static DEVICE_ATTR(region, 0777, morrison_kpd_region_show, morrison_kpd_region_store);

#endif

static int keypad_leds_probe (struct platform_device *pdev)
{
	int ret;
#ifdef MORRISON_KPD_DEBUG
    int ret1;
#endif

    if (!machine_is_morrison()) {
        printk (KERN_ERR "%s: wrong machine; returning.\n", __FUNCTION__);
        return -EINVAL;
    }
	ret = morrison_kpd_register_leds (&pdev->dev);
    if (ret)
        return ret;
    data.dev = &pdev->dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend (&early_suspend_data);
#endif

#ifdef MORRISON_KPD_DEBUG
    ret1 = device_create_file (keypad_leds[0].dev, &dev_attr_pwm); 
    if (ret1) {
        printk (KERN_ERR "%s: unable to create pwm device file for %s: %d\n",
            __FUNCTION__, keypad_leds[0].name, ret1);
    } else {
        dev_set_drvdata (keypad_leds[0].dev, &keypad_leds);
    }
    ret1 = device_create_file (keypad_leds[0].dev, &dev_attr_region); 
    if (ret1) {
        printk (KERN_ERR "%s: unable to create region device file for %s: %d\n",
            __FUNCTION__, keypad_leds[0].name, ret1);
    } else {
        dev_set_drvdata (keypad_leds[0].dev, &keypad_leds);
    }
#endif
    atomic_set (&data.suspended, 0);
    if (als_is_dark ()) {
        atomic_set (&data.als_zone, 0);
    } else {
        atomic_set (&data.als_zone, 1);
    }
    printk (KERN_INFO "%s: ALS zone = %d\n", __FUNCTION__, 
        atomic_read (&data.als_zone));
    lm3535_register_als_callback (keypad_als_changed, 0);
	return 0;
}

static int keypad_leds_remove (struct platform_device *pdev)
{
#ifdef MORRISON_KPD_DEBUG
    device_remove_file (keypad_leds[0].dev, &dev_attr_pwm); 
    device_remove_file (keypad_leds[0].dev, &dev_attr_region); 
#endif
    morrison_kpd_unregister_leds ();

	return 0;
}

static void keypad_leds_dev_release (struct device *dev)
{
}

static struct platform_device keypad_leds_device = {
    .name = "keypad-leds",
    .dev = {
        .release = keypad_leds_dev_release,
    }
};

static struct platform_driver keypad_leds_driver = {
	.probe		= keypad_leds_probe,
	.remove		= keypad_leds_remove,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= keypad_leds_suspend,
	.resume		= keypad_leds_resume,
#endif
#endif
	.driver		= {
		.name		= "keypad-leds",
        .owner = THIS_MODULE,
	},
};


static int __init keypad_leds_init(void)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
	ret = platform_driver_register (&keypad_leds_driver);
    if (ret) {
        printk (KERN_ERR "%s: platform_driver_register returned %d\n",
            __FUNCTION__, ret);
    }
    ret = platform_device_register (&keypad_leds_device);
    if (ret) {
        printk (KERN_ERR "%s: platform_device_register returned %d\n",
            __FUNCTION__, ret);
    }

    return ret;
}

static void __exit keypad_leds_exit(void)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
	platform_device_unregister (&keypad_leds_device);
	platform_driver_unregister (&keypad_leds_driver);
}

module_init(keypad_leds_init);
module_exit(keypad_leds_exit);

MODULE_AUTHOR("Alina Yakovleva qvdh43@motorola.com");
MODULE_DESCRIPTION("Morrison Keypad LED driver");
MODULE_LICENSE("GPL");
