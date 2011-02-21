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
 * PMIC7540 Keypad LED Driver for Android
 *
 * Alina Yakovleva qvdh43@motorola.com
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include "als.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define PM7540_KPD_LEDS_DEBUG

#define MODULE_NAME "leds_pm7540_kpd"


#define DEFAULT_MAX 5
#define DEFAULT_DIM 2
#define DEFAULT_SLEEP 50
#define DEFAULT_VREG_LEVEL 4100

static unsigned do_trace = 0;
module_param(do_trace, uint, 0644);

#define printk_info(fmt,args...) if (do_trace) printk(KERN_INFO fmt, ##args)

static unsigned als_zone_max[NUM_ALS_ZONES] = {
    DEFAULT_DIM, DEFAULT_MAX, DEFAULT_MAX, DEFAULT_MAX, DEFAULT_MAX, 
    DEFAULT_MAX};
static unsigned als_zone_max_prc[NUM_ALS_ZONES] = {4, 0, 0, 0, 0, 0};
module_param_array(als_zone_max, uint, NULL, 0644);

struct pm7540_kpd_data {
    int bvalue; // Current brightness value
    int ubvalue; // Current user brightness value
    int sleep;  // Sleep in ms between steps
    atomic_t als_zone;  // Current ALS zone
    unsigned vreg_level;  // Vreg voltage level
    unsigned is_prc;
};

static DEFINE_MUTEX(pm7540_mutex);
static struct pm7540_kpd_data kpd_data = {
    .bvalue = 0,
    .ubvalue = 0,
    .sleep = DEFAULT_SLEEP,
    .vreg_level = DEFAULT_VREG_LEVEL,
    .is_prc = false,
};
module_param_named(sleep_step, kpd_data.sleep, int, 0664);
module_param_named(vreg, kpd_data.vreg_level, int, 0664);
module_param_named(brightness, kpd_data.bvalue, int, 0444);
module_param_named(user_brightness, kpd_data.ubvalue, int, 0444);
module_param_named(is_prc, kpd_data.is_prc, uint, 0444);

#ifdef PM7540_KPD_LEDS_DEBUG
static atomic_t pm7540_debug;
#endif

static int pm7540_kpd_leds_suspend(struct platform_device *dev,
    pm_message_t state);
static int pm7540_kpd_leds_resume(struct platform_device *dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pm7540_kpd_leds_early_suspend (struct early_suspend *h);
static void pm7540_kpd_leds_late_resume (struct early_suspend *h);

struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = pm7540_kpd_leds_early_suspend,
    .resume = pm7540_kpd_leds_late_resume,
};

static void pm7540_kpd_leds_early_suspend (struct early_suspend *h)
{
    pm7540_kpd_leds_suspend (NULL, PMSG_SUSPEND);
}

static void pm7540_kpd_leds_late_resume (struct early_suspend *h)
{
    pm7540_kpd_leds_resume (NULL);
}

#endif

static void pm7540_kpd_do_brightness_set (unsigned value)
{
	static struct vreg *vreg = NULL;
	int ret;

    printk_info ("%s: %d\n", __FUNCTION__, value);
    if (vreg == NULL)
        vreg = vreg_get (0, "boost");
    if (value)
        ret = vreg_enable (vreg);
    else
        ret = vreg_disable (vreg);

	if (ret) {
		printk (KERN_ERR "%s: vreg_%s(boost) failed (%d)\n",
           __FUNCTION__, value ? "enable" : "disable", ret);
        return;
    }
    if (value) {
        if ((ret = vreg_set_level (vreg, kpd_data.vreg_level))) {
            printk (KERN_ERR "%s: vreg_set_level(boost, %d) failed (%d)\n",
               __FUNCTION__, kpd_data.vreg_level, ret);
            return;
        }
    }

    printk_info ("%s: brightness %d => %d\n", __FUNCTION__,
        kpd_data.bvalue, value);
    ret = 0;
    while (kpd_data.bvalue != value) {
        if (kpd_data.bvalue < value)
            kpd_data.bvalue++;
        else
            kpd_data.bvalue--;
        ret |= set_led_intensity (LED_KEYPAD, kpd_data.bvalue);
        mdelay (kpd_data.sleep);
    }
    if (ret) {
        printk (KERN_ERR "%s: msm_rpc_call (PM_SET_LED_INTENSITY) error %d\n",
            __FUNCTION__, ret);
    }
}

static void pm7540_kpd_brightness_set (struct led_classdev *led_cdev,
    enum led_brightness value)
{
#ifdef PM7540_KPD_LEDS_DEBUG
    if (atomic_read (&pm7540_debug)) {
        printk (KERN_ERR "%s: %d, in debug mode, ignoring\n",
            __FUNCTION__, value);
        return;
    }
#endif

    /* We don't check ALS zone; keypad is always illuminated */
    printk_info ("%s: %d\n", __FUNCTION__, value);
    mutex_lock (&pm7540_mutex);
    kpd_data.ubvalue = value;
    pm7540_kpd_do_brightness_set (value ?
        als_zone_max[atomic_read (&kpd_data.als_zone)] : value);
    mutex_unlock (&pm7540_mutex);
}

static struct led_classdev pm7540_kpd_leds = {
	.name			= "keyboard-backlight",
	.brightness_set		= pm7540_kpd_brightness_set,
};

static struct led_classdev pm7540_kpd_tcmd_leds = {
	.name			= "keyboard-tcmd",
	.brightness_set		= pm7540_kpd_brightness_set,
};

#ifdef CONFIG_PM
static int pm7540_kpd_leds_suspend (struct platform_device *dev,
    pm_message_t state)
{
#ifdef PM7540_KPD_LEDS_DEBUG
    if (atomic_read (&pm7540_debug)) {
        printk (KERN_ERR "%s: state = %d, in debug mode, ignoring\n",
            __FUNCTION__, state.event);
        return 0;
    }
#endif
	led_classdev_suspend (&pm7540_kpd_leds);
    printk (KERN_INFO "%s: driver suspended, event = %d\n",
        __FUNCTION__, state.event);
	return 0;
}

static int pm7540_kpd_leds_resume (struct platform_device *dev)
{
#ifdef PM7540_KPD_LEDS_DEBUG
    if (atomic_read (&pm7540_debug)) {
        printk (KERN_ERR "%s: in debug mode, ignoring\n",
            __FUNCTION__);
        return 0;
    }
#endif
	led_classdev_resume (&pm7540_kpd_leds);
    printk (KERN_INFO "%s: driver resumed\n", __FUNCTION__);
	return 0;
}
#endif

#ifdef PM7540_KPD_LEDS_DEBUG
#define USAGE \
"Usage: echo <command> [<arg>] > debug\n\
Commands can be:\n\
    sleep [<ms>] - change ramp sleep time to <ms> or to default 50ms;\n\
    max [<val>] - change max brightness to <val> (1 to 15) or to default of 5;\n\
    dim [<val>] - change dim brightness to <val> (1 to 15) or to default of 3;\n\
    vreg [<val>] - change vreg boost level to <val> uA or to default of 4500 uA;\n\
    start - put the driver into debug mode.  In debug mode the driver will ignore user requests from brightness file, will respond only to debug file and will not go into sleep mode.\n\
    stop - take the driver out of debug mode.\n\
    <brightness> - change the brightness, can be from 0 to 15.\n\n"

static ssize_t pm7540_kpd_debug_show (struct device *dev,
				      struct device_attribute *attr, char *buf)
{
    sprintf (buf, "%s\nsleep=%d, vreg=%d, in %s mode\n",
        USAGE, kpd_data.sleep,
        kpd_data.vreg_level, atomic_read (&pm7540_debug) ? "debug" : "normal");
    return strlen(buf) + 1;
}

static ssize_t pm7540_kpd_debug_store (struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
    unsigned value;
    char *ptr;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }

    if (strstr (buf, "sleep")) {
        ptr = strpbrk (buf, "0123456789");
        if (ptr) {
            value = simple_strtoul (ptr, NULL, 10);
            printk (KERN_ERR "%s: changing sleep to %d ms\n",
                __FUNCTION__, value);
            kpd_data.sleep = value;
        } else {
            printk (KERN_ERR "%s: changing sleep to default %d\n",
                __FUNCTION__, DEFAULT_SLEEP);
            kpd_data.sleep = DEFAULT_SLEEP;
        }
        return size;
    }
    if (strstr (buf, "vreg")) {
        ptr = strpbrk (buf, "0123456789");
        if (ptr) {
            value = simple_strtoul (ptr, NULL, 10);
            printk (KERN_ERR "%s: changing vreg level to %d\n",
                __FUNCTION__, value);
            kpd_data.vreg_level = value;
            return size;
        } else {
            printk (KERN_ERR "%s: changing vreg level to default %d\n",
                __FUNCTION__, DEFAULT_VREG_LEVEL);
            kpd_data.vreg_level = DEFAULT_VREG_LEVEL;
            return size;
        }
    }
    if (strstr (buf, "start")) {
        atomic_set (&pm7540_debug, 1);
        printk (KERN_ERR "%s: enter debug mode\n", __FUNCTION__);
        return size;
    }
    if (strstr (buf, "stop")) {
        atomic_set (&pm7540_debug, 0);
        printk (KERN_ERR "%s: exit debug mode\n", __FUNCTION__);
        return size;
    }
    if (atomic_read (&pm7540_debug)) {
        value = simple_strtoul (buf, NULL, 10);
        if (value > 150) {
            printk (KERN_ERR "%s: invalid value %d\n", __FUNCTION__, value);
            return -EINVAL;
        }
        printk (KERN_ERR "%s: set brightness to %d\n", __FUNCTION__, value);
        mutex_lock (&pm7540_mutex);
        pm7540_kpd_do_brightness_set (value);
        mutex_unlock (&pm7540_mutex);
        return size;
    } else {
        printk (KERN_ERR "%s: received unknown command: \"%s\"\n",
            __FUNCTION__, buf);
        return -EINVAL;
    }
    return size;
}

static DEVICE_ATTR(debug, 0777, pm7540_kpd_debug_show, pm7540_kpd_debug_store);
#endif

static void pm7540_kpd_als_changed (unsigned old_zone, unsigned zone,
    uint32_t cookie)
{
    if (zone >= NUM_ALS_ZONES) {
        printk (KERN_ERR "%s: invalid ALS zone %d\n", __FUNCTION__, zone);
        return;
    }
    mutex_lock (&pm7540_mutex);
    if (als_zone_max[zone] == 0) {
        // If brightness for this zone is 0 then turn it off regardless
        // of framework setting
        pm7540_kpd_do_brightness_set (als_zone_max[zone]);
    } else if (kpd_data.ubvalue) {
        // Turn it on only if framework has it turned on
        if (kpd_data.is_prc ||
            (!kpd_data.is_prc && (zone > old_zone))) {
            pm7540_kpd_do_brightness_set (als_zone_max[zone]);
        }
    }
    atomic_set (&kpd_data.als_zone, zone);
    mutex_unlock (&pm7540_mutex);
    printk_info ("%s: ALS zone changed: %d => %d\n",
        __FUNCTION__, old_zone, zone);
}

extern char *board_model_name (void);
static int pm7540_kpd_leds_probe (struct platform_device *pdev)
{
	int ret, ret1;
    char *board_model;
#ifdef CONFIG_MACH_MOT
    if (machine_is_morrison()) {
        printk (KERN_ERR "%s: morrison; returning.\n", __FUNCTION__);
        return -EINVAL;
    }
#endif
#if 0
	if (!machine_is_motus ()) {
        printk (KERN_ERR "%s: not motus; returning.\n", __FUNCTION__);
        return -EINVAL;
    }
#endif
    // See if this is ME600 for PRC
    board_model = board_model_name();
    if (board_model && !strcasecmp (board_model, "ME600")) {
        printk (KERN_INFO "%s: board model is ME600; changing brightness settings\n", __FUNCTION__);
        memcpy (als_zone_max, als_zone_max_prc, sizeof (als_zone_max));
        kpd_data.is_prc = true;
    }
    atomic_set (&kpd_data.als_zone, 5);
	ret = led_classdev_register (&pdev->dev, &pm7540_kpd_leds);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s error %d\n",
            __FUNCTION__, pm7540_kpd_leds.name, ret);
        return ret;
    }
	ret = led_classdev_register (&pdev->dev, &pm7540_kpd_tcmd_leds);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s error %d\n",
            __FUNCTION__, pm7540_kpd_tcmd_leds.name, ret);
    }
#ifdef PM7540_KPD_LEDS_DEBUG
    ret1 = device_create_file (pm7540_kpd_leds.dev, &dev_attr_debug);
    if (ret1) {
        printk (KERN_INFO "%s: unable to create device file for %s: %d\n",
            __FUNCTION__, pm7540_kpd_leds.name, ret1);
    } else {
        dev_set_drvdata (pm7540_kpd_leds.dev, &pm7540_kpd_leds);
    }
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend (&early_suspend_data);
#endif

    lm3535_register_als_callback (pm7540_kpd_als_changed, 0);

	return ret;
}

static int pm7540_kpd_leds_remove (struct platform_device *pdev)
{
	led_classdev_unregister (&pm7540_kpd_leds);
	led_classdev_unregister (&pm7540_kpd_tcmd_leds);
#ifdef PM7540_KPD_LEDS_DEBUG
    device_remove_file (pm7540_kpd_leds.dev, &dev_attr_debug);
#endif

	return 0;
}

static struct platform_device pm7540_kpd_leds_device = {
    .name = "pm7540-kpd-leds"
};

static struct platform_driver pm7540_kpd_leds_driver = {
	.probe		= pm7540_kpd_leds_probe,
	.remove		= pm7540_kpd_leds_remove,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= pm7540_kpd_leds_suspend,
	.resume		= pm7540_kpd_leds_resume,
#endif
#endif
	.driver		= {
		.name		= "pm7540-kpd-leds",
        .owner = THIS_MODULE,
	},
};

static int __init pm7540_kpd_leds_init(void)
{
    int ret;

	ret = platform_driver_register(&pm7540_kpd_leds_driver);
    printk (KERN_ERR "%s: platform_driver_register returned %d\n",
        __FUNCTION__, ret);
    ret = platform_device_register(&pm7540_kpd_leds_device);
    printk (KERN_ERR "%s: platform_device_register returned %d\n",
        __FUNCTION__, ret);

    return ret;
}

static void __exit pm7540_kpd_leds_exit(void)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
	platform_device_unregister(&pm7540_kpd_leds_device);
	platform_driver_unregister(&pm7540_kpd_leds_driver);
}

module_init(pm7540_kpd_leds_init);
module_exit(pm7540_kpd_leds_exit);


MODULE_AUTHOR("Alina Yakovleva qvdh43@motorola.com");
MODULE_DESCRIPTION("PM7540 Keypad LED Driver");
MODULE_LICENSE("GPL");
