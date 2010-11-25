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
 * Fairchild FAN5646 blinking LED Driver for Android
 *
 * Alina Yakovleva qvdh43@motorola.com
 */

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <asm/gpio.h>
#include <mach/vreg.h>
//#include "tinywire.h"

#if defined(CONFIG_MACH_CALGARY) || defined(CONFIG_MACH_PITTSBURGH)
#include <mach/board.h>
#include "board-mot.h" //GPIO/I2C addresses

#define FAN5646_GPIO MSG_IND_LED_CTRL
#else /* defined(CONFIG_MACH_CALGARY) || defined(CONFIG_MACH_PITTSBURGH) */
#define FAN5646_GPIO 100
#endif /* defined(CONFIG_MACH_CALGARY) || defined(CONFIG_MACH_PITTSBURGH) */

#define FAN5646_SLEW1_REG   0
#define FAN5646_PULSE1_REG  0x1
#define FAN5646_SLEW2_REG   0x2
#define FAN5646_PULSE2_REG  0x3
#define FAN5646_CONTROL_REG 0x4

#define FAN5646_ISET_5mA  0
#define FAN5646_ISET_10mA 0x40
#define FAN5646_ISET_15mA 0x80
#define FAN5646_ISET_20mA 0xC0

#define FAN5646_FOLLOW 0x1
#define FAN5646_PLAY   0x2
#define FAN5646_SLOW   0x4

#define FAN5646_MAX_TON        1600
#define FAN5646_MAX_TOFF       4800
#define FAN5646_MAX_TRISE      1550
#define FAN5646_MAX_TFALL      1550
#define FAN5646_MAX_ON         4700 // tRise + tFall + tOn
#define FAN5646_MAX_ON_SLOW    6300 // tRise + tFall + 2 * tOn
#define FAN5646_MAX_OFF        FAN5646_MAX_TOFF
#define FAN5646_MAX_OFF_SLOW   9600

#define NUM_LEDS 1
#define TRESET 110 // Sleep time in us

#define DEFAULT_UP   45
#define DEFAULT_DOWN 45

#define MODULE_NAME "leds_fan5646"
enum {
    TRACE_BRIGHTNESS = 0x1,
    TRACE_TINYWIRE = 0x2,
};

static unsigned do_trace = 0;
module_param(do_trace, uint, 0644);

#define printk_br(fmt,args...) if (do_trace & TRACE_BRIGHTNESS) printk(KERN_INFO fmt, ##args)
#define printk_tw(fmt,args...) if (do_trace & TRACE_TINYWIRE) printk(KERN_INFO fmt, ##args)

struct fan5646 {
    struct led_classdev leds[NUM_LEDS];
    unsigned is_reg[NUM_LEDS];
    struct device *dev;
    __u8 default_current;
    unsigned tsleep;
    spinlock_t lock;
    struct vreg *vr;
};

static void fan5646_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness bvalue);
static void fan5646_set_pulse (unsigned msOn, unsigned msOff,
    unsigned ramp_up, unsigned ramp_down, __u8 *slew, __u8 *pulse);
static int fan5646_blink_set (struct led_classdev *led_cdev,
    unsigned long *delay_on, unsigned long *delay_off);
static void tinywire_write_reg (int gpio, __u8 reg, __u8 value, unsigned tsleep);
static void tinywire_send_bit (int gpio, __u8 bit, unsigned tsleep);

static struct fan5646 fan5646_data = {
    .leds = {
        {
            .name            = "messaging",
            .brightness_set  = fan5646_brightness_set,
            .blink_set       = fan5646_blink_set,
        },
    },
    .default_current = FAN5646_ISET_5mA,
    .tsleep = 7,
    .lock = SPIN_LOCK_UNLOCKED,
    .vr = NULL,
};

static inline void usleep (unsigned int usecs)
{
    unsigned long tmout;

    tmout = jiffies + usecs_to_jiffies (usecs);
    while (time_before (jiffies, tmout)) {
        //cpu_relax ();
    }
}

void inline tinywire_send_bit (int gpio, __u8 bit, unsigned tsleep)
{
    //printk (KERN_ERR "%s: gpio=%d, bit=%d, tsleep=%d\n",
    //    __FUNCTION__, gpio, bit, tsleep);
    if (bit == 0) {
        gpio_set_value (gpio, 1);
        udelay (tsleep);
        gpio_set_value (gpio, 0);
        udelay (tsleep * 4);
    } else {
        gpio_set_value (gpio, 1);
        udelay (tsleep * 4);
        gpio_set_value (gpio, 0);
        udelay (tsleep);
    }
}

void inline tinywire_send_reset (int gpio)
{
    gpio_set_value (gpio, 0);
    udelay (TRESET);
}

void inline tinywire_send_exec (int gpio)
{
    gpio_set_value (gpio, 1);
    udelay (TRESET);
}

void tinywire_write_reg (int gpio, __u8 reg, __u8 value, unsigned tsleep)
{
    int i;
    __u8 mask = 0x1;

    printk_tw ("%s: reg=0x%x, value=0x%0x, tsleep=%dus\n", 
        __FUNCTION__, reg, value, tsleep);
    /* Register address is 3 bits.  Send it LSB first */
    for (i = 0; i < 3; i++) {
        tinywire_send_bit (gpio, reg & (mask << i), tsleep);
    }
    /* Now send data LSB first */
    for (i = 0; i < 8; i++) {
        tinywire_send_bit (gpio, value & (mask << i), tsleep);
    }
    /* Send STOP bit */
    tinywire_send_bit (gpio, 0, tsleep);
    /* Wait for TRESET so that it stays IDLE */
    udelay (TRESET);
}

static void fan5646_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value)
{
    __u8 ctrl_value;
    unsigned long flags;

    printk_br ("%s: %d\n", __FUNCTION__, value);

#if !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH)
    if (value)
        vreg_enable (fan5646_data.vr);
    else
        vreg_disable (fan5646_data.vr);
#endif /* !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH) */

    spin_lock_irqsave (&fan5646_data.lock, flags);
    tinywire_send_reset (FAN5646_GPIO);
    if (value) {
        /* Set default current and follow bit and raise control */
        ctrl_value = fan5646_data.default_current | FAN5646_FOLLOW;
        tinywire_write_reg (FAN5646_GPIO, FAN5646_CONTROL_REG, ctrl_value,
            fan5646_data.tsleep);
        /* Clear second pulse or it will keep blinking */
        tinywire_write_reg (FAN5646_GPIO, FAN5646_PULSE1_REG, 0,
            fan5646_data.tsleep);
        tinywire_send_exec (FAN5646_GPIO);
    }
    spin_unlock_irqrestore (&fan5646_data.lock, flags);
}

static int fan5646_blink_set (struct led_classdev *led_cdev,
    unsigned long *delay_on, unsigned long *delay_off)
{
    __u8 ctrl_value = fan5646_data.default_current | FAN5646_PLAY;
    __u8 slew, pulse;
    unsigned long flags;

    printk_br ("%s: delay_on = %lu, delay_off = %lu\n",
        __FUNCTION__, *delay_on, *delay_off);
    if (*delay_on == 0 && *delay_off == 0) {
        *delay_on = 500;
        *delay_off = 500;
    }
#if !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH)
    vreg_enable (fan5646_data.vr);
#endif /* !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH) */
    spin_lock_irqsave (&fan5646_data.lock, flags);
    tinywire_send_reset (FAN5646_GPIO);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_CONTROL_REG, ctrl_value,
        fan5646_data.tsleep);
    fan5646_set_pulse (*delay_on, *delay_off, DEFAULT_UP, DEFAULT_DOWN,
        &slew, &pulse);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_SLEW1_REG, slew, 
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_PULSE1_REG, pulse,
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_SLEW2_REG, 0, 
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_PULSE2_REG, 0,
        fan5646_data.tsleep);
    tinywire_send_exec (FAN5646_GPIO);
    spin_unlock_irqrestore (&fan5646_data.lock, flags);
    return 0;
}

// No PM for messaging LED - it has to blink until cancelled by user
#if 0
#ifdef CONFIG_PM
static int fan5646_suspend (struct platform_device *dev, pm_message_t state)
{
    printk (KERN_ERR "%s: state = %d\n", __FUNCTION__, state);
    led_classdev_suspend (&fan5646_data.leds[0]);
    return 0;
}

static int fan5646_resume (struct platform_device *dev)
{
    printk (KERN_ERR "%s\n", __FUNCTION__);
    led_classdev_resume (&fan5646_data.leds[0]);
    return 0;
}
#endif
#endif

static void fan5646_set_pulse (unsigned msOn, unsigned msOff,
    unsigned ramp_up, unsigned ramp_down, __u8 *slew, __u8 *pulse)
{
    __u8 nRise, nFall, nOn, nOff;
    unsigned tRise, tFall, tOn, tOff;
    unsigned slow = 0;

    printk_br ("%s: msOn = %d, msOff = %d, ramp up = %d%%, down = %d%%\n",
        __FUNCTION__, msOn, msOff, ramp_up, ramp_down);
    *slew = 0;
    *pulse = 0;

    if (msOn == 0 && msOff == 0)
        return;
    /* We won't do slow for now */
    if (msOn > FAN5646_MAX_ON) {
        msOn = FAN5646_MAX_ON;
    }
    if (msOff > FAN5646_MAX_OFF) {
        msOff = FAN5646_MAX_OFF;
    }
    tOff = msOff;
    /* Now the blinking part 
     * msOn consists of 3 parts: tRise, tFall, and tOn.
     */
    if (ramp_up + ramp_down > 100) {
        printk (KERN_ERR "%s: incorrect percent up %d%%, percent down %d%%; resetting\n",
            __FUNCTION__, ramp_up, ramp_down);
        ramp_up = DEFAULT_UP;
        ramp_down = DEFAULT_DOWN;
    }
    tOn = (100 - ramp_up - ramp_down) * msOn / 100; 
    tRise = ramp_up * msOn / 100;
    tFall = ramp_down * msOn / 100;
    //tRise = tFall = (msOn - tOn) / 2;
    if (tRise > FAN5646_MAX_TRISE) {
        tOn += tRise - FAN5646_MAX_TRISE;
        tRise = FAN5646_MAX_TRISE;
    }
    if (tFall > FAN5646_MAX_TRISE) {
        tOn += tFall - FAN5646_MAX_TRISE;
        tFall = FAN5646_MAX_TRISE;
    }
    // Now we need to calculate nRise, nFall, nOn and nOff
    // tRise = 31 * nRise * 3.33 ms, same for tFall
    // nRise = tRise / 103.23
    nRise = tRise * 100 / 10323;
    if (nRise > 0xF)
        nRise = 0xF;
    if (nRise == 0 && ramp_up != 0)
        nRise = 1;
    nFall = tFall * 100 / 10323;
    if (nFall > 0xF)
        nFall = 0xF;
    if (nFall == 0 && ramp_down != 0)
        nFall = 1;

    *slew = nRise << 4 | nFall;

    /* Now tOn and tOff
     * tOn = (SLOW + 1) * nOn * 106.6
     * tOff = (SLOW + 1) * nOff * 320
     * nOn = tOn / ((SLOW + 1) * 106.6)
     * nOff = tOff / ((SLOW + 1) * 320)
     */
    nOn = tOn * 10 / ((slow + 1) * 1066);
    nOff = tOff / ((slow + 1) * 320);
    if (nOn > 0xF)
        nOn = 0xF;
    if (nOff > 0xF)
        nOff = 0xF;
    if (nOn == 0 && (ramp_up + ramp_down < 100))
        nOn = 1;
    if (nOff == 0 && msOff != 0)
        nOff = 1;
    *pulse = nOn << 4 | nOff;

    printk_br ("%s: tRise = %d, tFall = %d, tOn = %d, tOff = %d, slow = %d\n",
        __FUNCTION__, tRise, tFall, tOn, tOff, slow);
    printk_br ("%s: nRise = 0x%x, nFall = 0x%x, nOn = 0x%x, nOff = 0x%x\n",
        __FUNCTION__, nRise, nFall, nOn, nOff);
}

static ssize_t fan5646_blink_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t fan5646_blink_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned msOn = 0, msOff = 0, bvalue = 0;
    unsigned msOn1 = 0, msOff1 = 0;
    int n;
    struct led_classdev *led_cdev;
    __u8 ctrl_value, slew, pulse, slew1 = 0, pulse1 = 0;
    int ramp_up = DEFAULT_UP;
    int ramp_down = DEFAULT_DOWN;
    int ramp_up1 = DEFAULT_UP;
    int ramp_down1 = DEFAULT_DOWN;
    char *ptr;
    unsigned long flags;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    led_cdev = dev_get_drvdata (dev);

    /* The format is: "brightness msOn msOff" */
    n = sscanf (buf, "%d %d %d %d %d", &bvalue, &msOn, &msOff, &msOn1, &msOff1);

    if (n != 3 && n != 5) {
        printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    printk_br ("%s: %s, b = %d, msOn = %d, msOff = %d, msOn1 = %d, msOff1 = %d\n",
        __FUNCTION__, led_cdev->name, bvalue, msOn, msOff, msOn1, msOff1);

    if (bvalue == 0 || (bvalue != 0 && msOn == 0 && msOff == 0)) {
        fan5646_brightness_set (led_cdev, bvalue);
        return size;
    }

    /* Now see if ramp values are there */
    ptr = strstr (buf, "ramp");
    if (ptr) {
        ptr = strpbrk (ptr, "0123456789");
        if (!ptr) {
            printk (KERN_ERR "%s: inavlid command (ramp): %s\n", 
                __FUNCTION__, buf);
            return -EINVAL;
        }
        n = sscanf (ptr, "%d %d %d %d", 
            &ramp_up, &ramp_down, &ramp_up1, &ramp_down1);
        if (n < 2) {
            printk (KERN_ERR "%s: inavlid command (ramp): %s\n", 
                __FUNCTION__, buf);
            return -EINVAL;
        }
        if (ramp_up < 0)
            ramp_up = DEFAULT_UP;
        if (ramp_down < 0)
            ramp_down = DEFAULT_DOWN;
        if (ramp_up1 < 0)
            ramp_up1 = DEFAULT_UP;
        if (ramp_down1 < 0)
            ramp_down1 = DEFAULT_DOWN;
        if (ramp_up + ramp_down > 100 || ramp_up1 + ramp_down1 > 100) {
            printk (KERN_ERR "%s: inavlid ramp times: %d%% up, %d%% down, %d%% up1, %d%% down1\n", 
                __FUNCTION__, ramp_up, ramp_down, ramp_up1, ramp_down1);
            return -EINVAL;
        }
    }
    printk_br ("%s: %s, ramp up = %d%%, ramp down = %d%%, ramp up1 = %d%%, ramp down1 = %d%%\n",
        __FUNCTION__, led_cdev->name, ramp_up, ramp_down, ramp_up1, ramp_down1);

#if !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH)
    vreg_enable (fan5646_data.vr);
#endif /* !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH) */
    fan5646_set_pulse (msOn, msOff, ramp_up, ramp_down, &slew, &pulse);
    fan5646_set_pulse (msOn1, msOff1, ramp_up1, ramp_down1, &slew1, &pulse1);
    ctrl_value = fan5646_data.default_current | FAN5646_PLAY;

    spin_lock_irqsave (&fan5646_data.lock, flags);
    tinywire_send_reset (FAN5646_GPIO); 
    tinywire_write_reg (FAN5646_GPIO, FAN5646_SLEW1_REG, slew, 
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_PULSE1_REG, pulse,
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_SLEW2_REG, slew1, 
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_PULSE2_REG, pulse1,
        fan5646_data.tsleep);
    tinywire_write_reg (FAN5646_GPIO, FAN5646_CONTROL_REG, ctrl_value,
        fan5646_data.tsleep);
    tinywire_send_exec (FAN5646_GPIO);
    spin_unlock_irqrestore (&fan5646_data.lock, flags);
    return size;
}

static DEVICE_ATTR(blink, 0777, fan5646_blink_show, fan5646_blink_store);

static ssize_t fan5646_settings_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    sprintf (buf, "current = %dmA, timing = %d\n", 
        (fan5646_data.default_current + 1) * 5, fan5646_data.tsleep);
    return strlen(buf) + 1;
}

static ssize_t fan5646_settings_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned value = 0;
    unsigned type = 0;
    char *ptr;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }

    if (strstr (buf, "current")) {
        type = 1;
    } else if (strstr (buf, "timing")) {
        type = 2;
    } else {
        printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    ptr = strpbrk (buf, "0123456789");
    if (ptr)
        value = simple_strtoul (ptr, NULL, 10);
    else {
        printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }

    switch (type) {
        case 1:
            switch (value) {
                case 5: fan5646_data.default_current = FAN5646_ISET_5mA;
                    break;
                case 10: fan5646_data.default_current = FAN5646_ISET_10mA;
                    break;
                case 15: fan5646_data.default_current = FAN5646_ISET_15mA;
                    break;
                case 20: fan5646_data.default_current = FAN5646_ISET_20mA;
                    break;
                default:
                    printk (KERN_ERR "%s: inavlid current value: %d\n", 
                        __FUNCTION__, value);
                    return -EINVAL;
            }
            printk (KERN_INFO "%s: changing current to %dmA\n",
                __FUNCTION__, value);
            break;
        case 2:
            printk (KERN_INFO "%s: changing timing to %dus\n",
                __FUNCTION__, value);
            fan5646_data.tsleep = value;
            break;
        default:
            printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
            return -EINVAL;
    }

    return size;
}

static DEVICE_ATTR(settings, 0777, fan5646_settings_show, fan5646_settings_store);

static ssize_t fan5646_prot_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t fan5646_prot_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned reg, value = 0;
    __u8 _reg, _value = 0;
    int n;
    unsigned long flags;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }

    n = sscanf (buf, "%d %d", &reg, &value);
    _reg = (__u8)reg;
    _value = (__u8)value;

    if (n == 0) {
        printk (KERN_ERR "%s: inavlid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    if (n == 1) {
        printk (KERN_INFO "%s: %s CTRL\n", __FUNCTION__, 
            reg ? "raising" : "lowering");
        spin_lock_irqsave (&fan5646_data.lock, flags);
        if (reg)
            tinywire_send_exec (FAN5646_GPIO);
        else
            tinywire_send_reset (FAN5646_GPIO);
        spin_unlock_irqrestore (&fan5646_data.lock, flags);
    } else if (n == 2) {
        printk (KERN_INFO "%s: reg=%d, value=%d\n", 
            __FUNCTION__, _reg, _value);
        spin_lock_irqsave (&fan5646_data.lock, flags);
        tinywire_send_reset (FAN5646_GPIO);
        tinywire_write_reg (FAN5646_GPIO, _reg, _value, fan5646_data.tsleep);
        spin_unlock_irqrestore (&fan5646_data.lock, flags);
    }
    return size;
}

static DEVICE_ATTR(prot, 0777, fan5646_prot_show, fan5646_prot_store);

static void fan5646_remove_device_files (void)
{
    int i;

    for (i = 0; i < NUM_LEDS; i++) {
        device_remove_file (fan5646_data.leds[i].dev, &dev_attr_blink);
    }
    device_remove_file (fan5646_data.leds[0].dev, &dev_attr_prot);
    device_remove_file (fan5646_data.leds[0].dev, &dev_attr_settings);
}

static int fan5646_create_device_files (void)
{
    int i, ret;

    for (i = 0; i < NUM_LEDS; i++) {
        ret = device_create_file (fan5646_data.leds[i].dev, &dev_attr_blink);
        if (ret) {
            printk (KERN_ERR "%s: unable to create device file for %s: %d\n",
                __FUNCTION__, fan5646_data.leds[i].name, ret);
            fan5646_remove_device_files ();
            return ret;
        }
        dev_set_drvdata (fan5646_data.leds[i].dev, &fan5646_data.leds[i]);
    }
    ret = device_create_file (fan5646_data.leds[0].dev, &dev_attr_prot);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"prot\" device file for %s: %d\n",
            __FUNCTION__, fan5646_data.leds[0].name, ret);
        fan5646_remove_device_files ();
        return ret;
    }
    ret = device_create_file (fan5646_data.leds[0].dev, &dev_attr_settings);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"settings\" device file for %s: %d\n",
            __FUNCTION__, fan5646_data.leds[0].name, ret);
        fan5646_remove_device_files ();
        return ret;
    }
    return 0;
}

static void fan5646_unregister_leds (void)
{
    int i;

    for (i = 0; i < NUM_LEDS; i++) {
        if (fan5646_data.is_reg[i]) {
            led_classdev_unregister (&fan5646_data.leds[i]);
            fan5646_data.is_reg[i] = 0;
        }
    }
}

static int fan5646_register_leds (struct device *dev)
{
    int i;
    int ret;

    for (i = 0; i < NUM_LEDS; i++) {
        ret = led_classdev_register (dev, &fan5646_data.leds[i]);
        if (ret) {
            printk (KERN_ERR "%s: unable to register led %s: error %d\n",
                __FUNCTION__, fan5646_data.leds[i].name, ret);
            fan5646_unregister_leds ();
            return ret;
        } else {
            fan5646_data.is_reg[i] = 1;
        }
    }
    return 0;
}

static int fan5646_probe (struct platform_device *pdev)
{
    int ret;

    printk (KERN_INFO "%s: enter\n", __FUNCTION__);

#if defined(CONFIG_MACH_CALGARY) || !defined(CONFIG_MACH_MOT) || !defined(CONFIG_MACH_PITTSBURGH)
	//if (!machine_is_morrison () && !machine_is_zeppelin()){
	if (!machine_is_morrison ()){
        printk (KERN_ERR "%s: not morrison and not zeppelin; returning\n",
            __FUNCTION__);
        return -EINVAL;
    }
#endif
    /*
    ret = gpio_request (FAN5646_GPIO, "fan5646_ctrl");
    if (ret) {
        printk (KERN_ERR "%s: gpio_request(%d, fan5646_ctrl) error %d\n",
            __FUNCTION__, FAN5646_GPIO, ret);
        return ret;
    }
    gpio_tlmm_config(GPIO_CFG(FAN5646_GPIO, 0, GPIO_OUTPUT, GPIO_NO_PULL,
        GPIO_2MA), GPIO_ENABLE);

    ret = gpio_direction_output (FAN5646_GPIO, 1);
    if (ret) {
        printk (KERN_ERR "%s: gpio_direction_output(%d, 1) error %d\n",
            __FUNCTION__, FAN5646_GPIO, ret);
        gpio_free (FAN5646_GPIO);
        return ret;
    }
    */
    gpio_set_value (FAN5646_GPIO, 0);

#if !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH)
    fan5646_data.vr = vreg_get (0, "boost");
    if (fan5646_data.vr == NULL) {
        printk (KERN_ERR "%s: vreg_get (0, boost) failed\n", __FUNCTION__);
        return -ENOENT;
    }
#endif /* !defined(CONFIG_MACH_CALGARY) && !defined(CONFIG_MACH_PITTSBURGH) */

    fan5646_data.dev = &pdev->dev;
    ret = fan5646_register_leds (&pdev->dev);
    if (ret) {
        gpio_free (FAN5646_GPIO);
        return ret;
    }

    ret = fan5646_create_device_files ();
    if (ret) {
        gpio_free (FAN5646_GPIO);
        fan5646_unregister_leds ();
        return ret;
    }
    //hrtimer_init (&fan5646_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //fan5646_timer.function = fan5646_timer_func;

    return ret;
}

static int __devexit fan5646_remove(struct platform_device *pdev)
{
    gpio_set_value (FAN5646_GPIO, 0);
    gpio_free (FAN5646_GPIO);
    fan5646_remove_device_files ();
    fan5646_unregister_leds ();

    return 0;
}

static struct platform_device fan5646_device = {
    .name = "fan5646"
};

static struct platform_driver fan5646_driver = {
    .probe        = fan5646_probe,
    .remove       = __devexit_p(fan5646_remove),
#if 0
#ifdef CONFIG_PM
    .suspend    = fan5646_suspend,
    .resume        = fan5646_resume,
#endif
#endif
    .driver        = {
        .name        = "fan5646",
        .owner = THIS_MODULE,
    },
};


static int __init fan5646_init(void)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    ret = platform_driver_register(&fan5646_driver);
    printk (KERN_ERR "%s: platform_driver_register returned %d\n",
        __FUNCTION__, ret);
    ret = platform_device_register(&fan5646_device);
    printk (KERN_ERR "%s: platform_device_register returned %d\n",
        __FUNCTION__, ret);

    return ret;
}

static void __exit fan5646_exit(void)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    platform_device_unregister(&fan5646_device);
    platform_driver_unregister(&fan5646_driver);
}

module_init(fan5646_init);
module_exit(fan5646_exit);

MODULE_AUTHOR("Alina Yakovleva qvdh43@motorola.com");
MODULE_DESCRIPTION("Fairchild FAN5646 LED Driver");
MODULE_LICENSE("GPL");
