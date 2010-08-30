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

//#############################################################################
//#
//#   FILE NAME       : lm2756.c
//#   ORIGINATOR      : Alina Yakovleva
//#   DATE OF ORIGIN  : 08/01/2008
//#
//#---------------------------------- PURPOSE ---------------------------------
//#        Linux  driver for    LM2756 display backlight
//#----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <asm/gpio.h>
#include <asm/io.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

typedef unsigned char uint8;

/******************************************************************************
 *  LM2756 registers
 ******************************************************************************/
#define LM2756_ENABLE_REG                0x10
#define LM2756_OPTIONS_REG               0x20
#define LM2756_BRIGHTNESS_CTRL_REG_A     0xA0
#define LM2756_BRIGHTNESS_CTRL_REG_B     0xB0
#define LM2756_BRIGHTNESS_CTRL_REG_C     0xC0

static int lm2756_write_reg (unsigned reg, uint8 value, const char *caller);
static int lm2756_read_reg (unsigned reg, uint8 *value);
static void lm2756_do_brightness_set (enum led_brightness value, 
    unsigned do_ramp);
static int lm2756_set_ramp (struct i2c_client *client, 
    unsigned int on, unsigned int nsteps, unsigned int *rtime);
static int lm2756_enable (unsigned int on);
static int lm2756_probe(struct i2c_client *client);
static int lm2756_remove (struct i2c_client *client);
static void lm2756_work_func (struct work_struct *work);
static irqreturn_t lm2756_irq_handler (int irq, void *dev_id);
static void lm2756_brightness_set(struct led_classdev *led_cdev,
                enum led_brightness value);
static void lm2756_brightness_set_noramp(struct led_classdev *led_cdev,
                enum led_brightness value);
#ifdef CONFIG_ANDROID_POWER
static void lm2756_early_suspend (struct android_early_suspend *h);
static void lm2756_late_resume (struct android_early_suspend *h);
#endif
static int lm2756_suspend (struct i2c_client *client, pm_message_t mesg);
static int lm2756_resume (struct i2c_client *client);

/* LED class struct */
static struct led_classdev lm2756_led = {
    .name = "lcd-backlight",
    .brightness_set = lm2756_brightness_set,
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver lm2756_driver =
{
    .driver = {
        .name   = "lm2756",
    },
    .probe = lm2756_probe,
    .remove  = lm2756_remove,
#ifndef CONFIG_ANDROID_POWER
    .suspend    = lm2756_suspend,
    .resume     = lm2756_resume,
#endif
};

struct lm2756 {
    uint16_t addr;
    struct i2c_client *client;
    unsigned initialized;
    unsigned enabled;
    int last_brightness;
    int nramp;
#ifdef CONFIG_ANDROID_POWER
    struct android_early_suspend early_suspend;
#endif
};
static DEFINE_MUTEX(lm2756_mutex);



static struct lm2756 lm2756_data = {
    .nramp = 4,
    .last_brightness = 255,
};

#ifdef CONFIG_ANDROID_POWER
static struct android_early_suspend lm2756_early_suspend_data = {
    .level = ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
    .suspend = lm2756_early_suspend,
    .resume = lm2756_late_resume,
};
#endif


static int lm2756_read_reg (unsigned reg, uint8 *value)
{
    struct i2c_client *client = lm2756_data.client;
    uint8 buf[1];
    int ret = 0;

    if (!value)
        return -EINVAL;
    buf[0] = reg;
    ret = i2c_master_send (client, buf, 1);
    if (ret > 0) {
        msleep_interruptible (1);
        ret = i2c_master_recv (client, buf, 1);
        if (ret > 0)
            *value = buf[0];
    }
    return ret;
}

static int lm2756_write_reg (unsigned reg, uint8 value, const char *caller)
{
    uint8 buf[2] = {reg, value};
    int ret = 0;

    printk (KERN_ERR "%s: writing 0x%X to reg 0x%X at addr 0x%X\n",
        caller, buf[1], buf[0], lm2756_data.client->addr);
    ret = i2c_master_send (lm2756_data.client, buf, 2);
    if (ret < 0)
        printk (KERN_ERR "%s: i2c_master_send error %d\n",
            caller, ret);
    return ret;
}

static void lm2756_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    printk (KERN_ERR "%s: 0x%x (%d)\n", __FUNCTION__, value, value);
    lm2756_do_brightness_set (value, 1);
}
EXPORT_SYMBOL(lm2756_brightness_set);

static void lm2756_do_brightness_set (enum led_brightness value, 
    unsigned do_ramp)
{
    struct i2c_client *client = lm2756_data.client;
    uint8 bvalue;
    int ret, nsteps;
    unsigned int total_time;

    if (!lm2756_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    mutex_lock (&lm2756_mutex);
    if (value > 255) // Value can't be greater than 255
        value = 255;

#ifdef LM2756_HACK
    /* HACK until we find where settings are stored */
    if (value > 0)
        value = 255;
    printk (KERN_ERR "%s: LM2756_HACK is defined; setting brightness to 255\n",
        __FUNCTION__);
#endif
    if (!lm2756_data.enabled) {
        lm2756_enable (1);
    }
    /* The chip supports brightness levels from 0 to 0x1F; need to scale.
       Shifting >> 3 should do */
    bvalue = (value >> 3);

    /* Calculate number of steps for ramping */
    nsteps = lm2756_data.last_brightness - value;
    if (nsteps < 0)
        nsteps *= -1;
    nsteps = nsteps >> 3;

    printk (KERN_ERR "%s: brightness %d => %d, nsteps = %d\n",
        __FUNCTION__, lm2756_data.last_brightness, value, nsteps);

    lm2756_set_ramp (client, do_ramp, nsteps, &total_time);

    ret = lm2756_write_reg (LM2756_BRIGHTNESS_CTRL_REG_A, 
        bvalue, __FUNCTION__);
    if (ret < 0) {
        mutex_unlock (&lm2756_mutex);
        return;
    }
#if 0
    ret = lm2756_read_reg (LM2756_BRIGHTNESS_CTRL_REG_A, &bvalue);
    if (ret > 0)
        printk (KERN_ERR "%s: brightness now is: 0x%X\n", __FUNCTION__, bvalue);
#endif

    if (value == 0) {
        //unsigned int msec = (unsigned int)total_time / 1000;
        unsigned int msec = 1000;

        /* Disable everything */
        if (do_ramp) {
            /* Wait for ramping to finish */
            printk (KERN_ERR "%s: msleep (%d)\n", __FUNCTION__, msec);
            msleep_interruptible (msec);
        }
        lm2756_enable (0);
    }

    lm2756_data.last_brightness = value;
    mutex_unlock (&lm2756_mutex);
}

/* This function is called by i2c_probe */
static int lm2756_probe (struct i2c_client *client)
{
    int ret = 0;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    /* We should be able to read and write byte data */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
            __FUNCTION__);
        return -ENOTSUPP;
    }
    printk (KERN_ERR "%s: I2C_FUNC_I2C is supported\n", __FUNCTION__);

    /* OK. For now, we presume we have a valid client. We now create the
       client structure, even though we cannot fill it completely yet. */

    lm2756_data.client = client;
    i2c_set_clientdata (client, &lm2756_data);

    /* Initialize chip */
    lm2756_enable (1);
    lm2756_data.initialized = 1;
    lm2756_data.last_brightness = 255;  // From bootloader

    /* Register LED class */
    ret = led_classdev_register (&client->adapter->dev, &lm2756_led);
    if (ret) {
        printk (KERN_ERR "%s: lcd-backlight led_classdev_register failed: %d\n",
           __FUNCTION__, ret);
        return ret;
    } else {
       printk (KERN_ERR "%s: lcd-backlight led_classdev_register success\n", 
           __FUNCTION__);
    }

    return 0;
}

struct lm2756_options_register {
    int rs      : 2;       // Ramp step
    int rd      : 2; 
};
/* Ramp times for Rev1 in microseconds */
static unsigned int lm2756_ramp[] = {100, 25000, 50000, 100000};

/* This function calculates ramp step time so that total ramp time is
 * 1.5 sec = 1500ms
 */
#define TOTAL_RAMP 1000000 // Total ramp time in microseconds
static int lm2756_set_ramp (struct i2c_client *client, 
    unsigned int on, unsigned int nsteps, unsigned int *rtime)
{
    int ret, i = 0;
    uint8 value = 0;
    unsigned int total_time = 0;
    struct lm2756_options_register reg;

    memset (&reg, 0, sizeof (reg));
    if (on) {
        /* Calculate the closest possible ramp time */
        for (i = 0; i < 4; i++) {
            total_time = nsteps * lm2756_ramp[i];
            if (total_time >= TOTAL_RAMP)
                break;
        }
        if (i > 0) {
            /* If previous value is closer */
            if (total_time - TOTAL_RAMP > 
                    TOTAL_RAMP - nsteps * lm2756_ramp[i-1]) {
                i--;
                total_time = nsteps * lm2756_ramp[i];
            }
        }
        reg.rs = i;
        memcpy (&value, &reg, sizeof (value));
    } 
     
    printk (KERN_ERR "%s: ramp = %s, ramp step = %d us (total = %d us)\n",
        __FUNCTION__, on ? "on" : "off", lm2756_ramp[i], total_time);
    if (rtime)
        *rtime = total_time;
    ret = lm2756_write_reg (LM2756_OPTIONS_REG, value, __FUNCTION__);

    return ret;
}

struct lm2756_enable_register {
    int en_a : 1;
    int en_b : 1;
    int en_c : 1;
    int sd_53 : 1;
    int sd_62 : 1;
    int en_53 : 1;
    int en_62 : 1;
};

static int lm2756_enable (unsigned int on)
{
    int ret = 0;
    uint8 value = 0;
    struct lm2756_enable_register reg = {
        .en_a = 1,
        .en_b = 0,
        .en_c = 0,
        .sd_53 = 0,
        .sd_62 = 0,
        .en_53 = 1,
        .en_62 = 1
    };

    if (on)
        memcpy (&value, &reg, sizeof (value));
    ret = lm2756_write_reg (LM2756_ENABLE_REG, value, __FUNCTION__);
    lm2756_data.enabled = on;
    return ret;
}

static int lm2756_remove (struct i2c_client *client)
{
    led_classdev_unregister (&lm2756_led);
    return 0;
}

static int lm2756_suspend (struct i2c_client *client, pm_message_t mesg)
{
    printk (KERN_ERR "%s: called with pm message %d\n", __FUNCTION__, mesg);
    led_classdev_suspend (&lm2756_led);
    return 0;
}

static int lm2756_resume (struct i2c_client *client)
{
    //struct lm2756 *data_ptr = i2c_get_clientdata (client);

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    mutex_lock (&lm2756_mutex);
    lm2756_enable (1);
    mutex_unlock (&lm2756_mutex);
    led_classdev_resume (&lm2756_led);

    return 0;
}

#ifdef CONFIG_ANDROID_POWER
static void lm2756_early_suspend (struct android_early_suspend *h)
{
    lm2756_suspend (lm2756_data.client, PMSG_SUSPEND);
}

static void lm2756_late_resume (struct android_early_suspend *h)
{
    lm2756_resume (lm2756_data.client);
}
#endif


static int __devinit lm2756_init (void)
{
    int ret;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    ret = i2c_add_driver (&lm2756_driver);
    if (ret) {
        printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n", 
            __FUNCTION__, ret);
    }
    return ret;
}

static void __exit lm2756_exit(void)
{
    i2c_del_driver (&lm2756_driver);
}

module_init(lm2756_init);
module_exit(lm2756_exit);

MODULE_DESCRIPTION("LM2756 DISPLAY BACKLIGHT DRIVER");
MODULE_LICENSE("GPL");
