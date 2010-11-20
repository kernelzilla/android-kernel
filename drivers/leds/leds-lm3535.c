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

// Linux  driver for    LM3535 display backlight

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
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
#include <linux/list.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "als.h"
#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_LM3535_ESD_RECOVERY
#include <mot/esd_poll.h>
#endif /* CONFIG_LM3535_ESD_RECOVERY */


#define LM3535_ADDRESS_HACK
#ifndef CONFIG_MACH_MOT
#if defined(LM3535_ADDRESS_HACK)
#include <mach/board.h>
#include "board-mot.h" //GPIO/I2C addresses
#endif
#endif

#define MODULE_NAME "leds_lm3535"
#ifndef CONFIG_MACH_MOT
#define SUPPORT_LM3535_2ALS  /* enable for LM3535-2ALS support */
#endif

/******************************************************************************
 *  LM3535 registers
 ******************************************************************************/
#define LM3535_ENABLE_REG                0x10
#define LM3535_CONFIG_REG                0x20
#define LM3535_OPTIONS_REG               0x30
#define LM3535_ALS_REG                   0x40
#define LM3535_ALS_CTRL_REG              0x50
#define LM3535_ALS_RESISTOR_REG          0x51
#ifdef CONFIG_LM3535_BUTTON_BL
#define LM3535_ALS_SELECT_REG            0x52
#endif
#define LM3535_BRIGHTNESS_CTRL_REG_A     0xA0
#define LM3535_BRIGHTNESS_CTRL_REG_B     0xB0
#define LM3535_BRIGHTNESS_CTRL_REG_C     0xC0
#define LM3535_ALS_ZB0_REG               0X60
#define LM3535_ALS_ZB1_REG               0X61
#define LM3535_ALS_ZB2_REG               0X62
#define LM3535_ALS_ZB3_REG               0X63
#define LM3535_ALS_Z0T_REG               0X70
#define LM3535_ALS_Z1T_REG               0X71
#define LM3535_ALS_Z2T_REG               0X72
#define LM3535_ALS_Z3T_REG               0X73
#define LM3535_ALS_Z4T_REG               0X74
#define LM3535_TRIM_REG                  0xD0

#define ALS_FLAG_MASK 0x08
#define ALS_ZONE_MASK 0x07
#define ALS_NO_ZONE   5
#define LM3535_TRIM_VALUE 0x68

#define LM3535_LED_MAX 0x7F  // Max brightness value supported by LM3535

/* Final revision CONFIG value */
#define CONFIG_VALUE 0x4C
#define CONFIG_VALUE_NO_ALS 0x0C

/* ALS Averaging time */
#define ALS_AVERAGING 0x50  // 1600ms, needs 2 ave. periods

/* Zone boundaries */
#ifdef CONFIG_MACH_MOT // Morrison, Zeppelin, Motus
static unsigned morrison_als_zb[] = {0x02, 0xBD, 0xFE, 0xFF};
static unsigned motus_als_zb[] =    {0x02, 0xCC, 0xEE, 0xEF};
static unsigned zeppelin_als_zb[] = {0x04, 0x42, 0x96, 0xFF};
static unsigned als_zb[4];
#else // Calgary
static unsigned als_zb[4] = {0x02, 0x34, 0xB4, 0xB4};
static unsigned als_zone_max[6] = {0xE3, 0xEB, 0xF6, 0xFC, 0xFC, 0xFC};
#endif
module_param_array(als_zb, uint, NULL, 0644);
static unsigned resistor_value = 0x30;
module_param(resistor_value, uint, 0644);
static unsigned pwm_value = 0x1;
#ifdef CONFIG_MACH_MOT
module_param(pwm_value, uint, 0644);
#endif

static unsigned als_sleep = 350;
module_param(als_sleep, uint, 0644);
#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_ESD_RECOVERY
static bool esd_polling = 0;
#endif /* CONFIG_LM3535_ESD_RECOVERY */
#endif
static unsigned ramp_time = 200000;
module_param(ramp_time, uint, 0644);
enum {
    TRACE_SUSPEND = 0x1,
    TRACE_ALS = 0x2,
    TRACE_BRIGHTNESS = 0x4,
    TRACE_WRITE = 0x8,
    TRACE_EVENT = 0x10,
};
#ifdef CONFIG_MACH_MOT
unsigned do_trace = TRACE_ALS | TRACE_SUSPEND | TRACE_BRIGHTNESS;
#else
unsigned do_trace = TRACE_ALS; // | TRACE_SUSPEND | TRACE_BRIGHTNESS | TRACE_WRITE;
#endif
module_param(do_trace, uint, 0644);

#define printk_write(fmt,args...) if (do_trace & TRACE_WRITE) printk(KERN_INFO fmt, ##args)
#define printk_br(fmt,args...) if (do_trace & TRACE_BRIGHTNESS) printk(KERN_INFO fmt, ##args)
#define printk_als(fmt,args...) if (do_trace & TRACE_ALS) printk(KERN_INFO fmt, ##args)
#define printk_suspend(fmt,args...) if (do_trace & TRACE_SUSPEND) printk(KERN_INFO fmt, ##args)
#define printk_event(fmt,args...) if (do_trace & TRACE_EVENT) printk(KERN_INFO fmt, ##args)

/* ALS callbacks */
static DEFINE_MUTEX(als_cb_mutex);
static LIST_HEAD(als_callbacks);
struct als_callback {
    als_cb cb;
#ifdef CONFIG_MACH_MOT
    uint32_t cookie;
#endif
    struct list_head entry;
};

struct lm3535_options_register_r1 {
    int rs      : 2;       // Ramp step
    int gt      : 2;       // Gain transition filter
    int rev     : 2;
};
/* Ramp times for Rev1 in microseconds */
static unsigned int lm3535_ramp_r1[] = {51, 13000, 26000, 52000};

struct lm3535_options_register_r2 {
    int rs_down : 2;
    int rs_up : 2;
    int gt : 2;
};
/* Ramp times for Rev2 in microseconds */
static unsigned int lm3535_ramp_r2[] = {6, 6000, 12000, 24000};

struct lm3535_options_register_r3 {
    int rs_down : 3;
    int rs_up : 3;
    int gt : 2;
};
/* Ramp times for Rev3 in microseconds */
static unsigned int lm3535_ramp_r3[] = 
    {6, 770, 1500, 3000, 6000, 12000, 25000, 50000};

#ifdef CONFIG_MACH_MOT
extern unsigned msm_fb_mddi_nv_get_manprodid (void);
extern void msm_fb_led_resumed(void);
#endif
static void lm3535_send_als_event (int zone);
static char *reg_name (int reg);
static int lm3535_configure (void);
static void lm3535_call_als_callbacks (unsigned old_zone, unsigned zone);
static void lm3535_set_options_r1 (uint8_t *buf, unsigned ramp);
static void lm3535_set_options_r2 (uint8_t *buf, unsigned ramp);
static void lm3535_set_options_r3 (uint8_t *buf, unsigned ramp);
static int lm3535_write_reg (unsigned reg, uint8_t value, const char *caller);
static int lm3535_read_reg (unsigned reg, uint8_t *value);
static int lm3535_set_ramp (struct i2c_client *client, 
    unsigned int on, unsigned int nsteps, unsigned int *rtime);
#ifdef CONFIG_MACH_MOT
static int lm3535_enable(struct i2c_client *client, unsigned int on);
#else
static int lm3535_enable (struct i2c_client *client, unsigned int display_on, unsigned int button_on);
#endif
static int lm3535_probe(struct i2c_client *client, 
    const struct i2c_device_id *id);
static int lm3535_setup (struct i2c_client *client);
static int lm3535_remove (struct i2c_client *client);
static void lm3535_work_func (struct work_struct *work);
static irqreturn_t lm3535_irq_handler (int irq, void *dev_id);
static void lm3535_brightness_set(struct led_classdev *led_cdev,
                enum led_brightness value);
#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
static void lm3535_button_brightness_set(struct led_classdev *led_cdev,
                enum led_brightness value);
#endif
#ifdef CONFIG_LM3535_ESD_RECOVERY
void lm3535_check_esd (void* arg);
#endif /* CONFIG_LM3535_ESD_RECOVERY */
#endif
/* static unsigned lm3535_read_als_zone (void); */
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3535_early_suspend (struct early_suspend *h);
static void lm3535_late_resume (struct early_suspend *h);
static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5,
    .suspend = lm3535_early_suspend,
    .resume = lm3535_late_resume,
};

#endif
static int lm3535_suspend (struct i2c_client *client, pm_message_t mesg);
static int lm3535_resume (struct i2c_client *client);
#endif

static void (*lm3535_set_options_f)(uint8_t *buf, unsigned ramp) = 
    lm3535_set_options_r1;
static unsigned int *lm3535_ramp = lm3535_ramp_r1;

/* LED class struct */
static struct led_classdev lm3535_led = {
    .name = "lcd-backlight",
    .brightness_set = lm3535_brightness_set,
};

/* LED class struct for no ramping */
static struct led_classdev lm3535_led_noramp = {
    .name = "lcd-nr-backlight",
    .brightness_set = lm3535_brightness_set,
};
#ifdef CONFIG_LM3535_BUTTON_BL
static struct led_classdev lm3535_led_button = {
    .name = "button-backlight",
    .brightness_set = lm3535_button_brightness_set,
};
#endif

static const struct i2c_device_id lm3535_id[] = {
    { "lm3535", 0 },
    { }
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver lm3535_driver =
{
    .driver = {
        .name   = "lm3535",
    },
    .id_table = lm3535_id,
    .probe = lm3535_probe,
    .remove  = lm3535_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = lm3535_suspend,
    .resume     = lm3535_resume,
#endif
};

#define LM3535_NUM_ZONES 6
struct lm3535 {
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *idev;
    unsigned initialized;
    unsigned enabled;
#ifndef CONFIG_MACH_MOT
    unsigned button_enabled;
#endif
    int use_irq;
    int revision;
    int nramp;
    atomic_t als_zone;     // Current ALS zone
    atomic_t bright_zone;  // Current brightness zone, diff. from ALS
    atomic_t use_als;      // Whether to use ALS
    atomic_t in_suspend;   // Whether the driver is in TCMD SUSPEND mode
    atomic_t do_als_config; // Whether to configure ALS averaging
    unsigned bvalue;       // Current brightness register value
    unsigned saved_bvalue; // Brightness before TCMD SUSPEND
    //struct hrtimer timer;
    struct work_struct  work;
};
static DEFINE_MUTEX(lm3535_mutex);

static struct lm3535 lm3535_data = {
    .nramp = 4,
    .bvalue = 0x79,
};

#if 0
unsigned lm3535_read_als_zone (void)
{
    uint8_t reg;
    int ret;

    if (lm3535_data.revision <= 1) {
        printk (KERN_ERR "%s: early revison, setting zone to 5 (no ALS)\n",
           __FUNCTION__);
        atomic_set (lm3535_data.als_zone, ALS_NO_ZONE);
        return ALS_NO_ZONE;
    }
    lm3535_read_reg (LM3535_CONFIG_REG, &reg);
    if (reg & 0x50) {
        ret = lm3535_read_reg (LM3535_ALS_REG, &reg);
        lm3535_data.als_zone = reg & ALS_ZONE_MASK;
        return lm3535_data.als_zone;
    } else {
        printk (KERN_ERR "%s: ALS is not enabled, CONFIG=0x%x, zone=5\n",
            __FUNCTION__, reg);
        lm3535_data.als_zone = 5;
        return 5;
    }
}
#endif
#ifdef CONFIG_MACH_MOT
int lm3535_register_als_callback(als_cb func, uint32_t cookie)
#else
int lm3535_register_als_callback (als_cb func)
#endif
{
    struct als_callback *c;

    //printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    c = kzalloc (sizeof (struct als_callback), GFP_KERNEL);
    if (c == NULL) {
        printk (KERN_ERR "%s: unable to register ALS callback: kzalloc\n",
            __FUNCTION__);
        return -ENOMEM;
    }
    c->cb = func;
#ifdef CONFIG_MACH_MOT
    c->cookie = cookie;
#endif
    mutex_lock (&als_cb_mutex);
    list_add (&c->entry, &als_callbacks);
    mutex_unlock (&als_cb_mutex);
    return 0;
}
EXPORT_SYMBOL(lm3535_register_als_callback);

void lm3535_unregister_als_callback (als_cb func)
{
    struct als_callback *c;

    if (!lm3535_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    mutex_lock (&als_cb_mutex);
    list_for_each_entry(c, &als_callbacks, entry) {
        if (c->cb == func) {
            list_del (&c->entry);
            mutex_unlock (&als_cb_mutex);
            return;
        }
    }
    mutex_unlock (&als_cb_mutex);
    printk (KERN_ERR "%s: callback 0x%x not found\n", 
        __FUNCTION__, (unsigned int)func);
}
EXPORT_SYMBOL(lm3535_unregister_als_callback);

unsigned lm3535_als_is_dark (void)
{
    unsigned zone;

    zone = atomic_read (&lm3535_data.als_zone);
    printk (KERN_ERR "%s: enter, zone = %d\n", 
        __FUNCTION__, zone);
    if (zone == 0 || zone == 5)
        return 1;
    else
        return 0;
}
EXPORT_SYMBOL(lm3535_als_is_dark);

static int lm3535_read_reg (unsigned reg, uint8_t *value)
{
    struct i2c_client *client = lm3535_data.client;
    uint8_t buf[1];
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

static int lm3535_write_reg (unsigned reg, uint8_t value, const char *caller)
{
    uint8_t buf[2] = {reg, value};
    int ret = 0;

    printk_write ("%s: writing 0x%X to reg 0x%X (%s) at addr 0x%X\n",
        caller, buf[1], buf[0], reg_name (reg), lm3535_data.client->addr);
    ret = i2c_master_send (lm3535_data.client, buf, 2);
    if (ret < 0)
        printk (KERN_ERR "%s: i2c_master_send error %d\n",
            caller, ret);
    return ret;
}

/* ALS Coefficients */
#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
static long als_z0_interim[] =  {0, 2000, 1483000, 637300000};
static long als_z0_interim_motus[] =  {0, 2000, 1483000, 637300000};
static long als_z0[] =          {-48, 22225, -616209, 587754883};
static long als_z0_motus[] =    {-48, 22225, -616209, 587754883};
static long als_z0_zeppelin[] = {15, -3792, 1882205, 535312172};
#else // Calgary
static long als_z0[] =  {0, 2000, 1483000, 637300000};
#endif
module_param_array(als_z0, long, NULL, 0644);

#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
static long als_z1_interim[] =  {0, -6500, 3677400, 687018600};
static long als_z1_interim_motus[] =  {0, -6500, 3677400, 687018600};
static long als_z1[] =          {24, -14596, 3987380, 659307205};
static long als_z1_motus[] =    {24, -14596, 3987380, 659307205};
static long als_z1_zeppelin[] =  {18, -12588, 3887115, 592862884};
#else // Calgary
static long als_z1[] =  {0, -6500, 3677400, 687018600};
#endif
module_param_array(als_z1, long, NULL, 0644);

#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
static long als_z2_interim[] =  {0, -5700, 2553000, 953424200};
static long als_z2_interim_motus[] =  {0, -5700, 2553000, 953424200};
static long als_z2[] =          {0, -5700, 2553000, 953424200};
static long als_z2_motus[] =    {0, -5700, 2553000, 953424200};
static long als_z2_zeppelin[] = {32, -17768, 3670047, 832704191};
#else // Calgary
static long als_z2[] =  {0, -5700, 2553000, 953424200};
#endif
module_param_array(als_z2, long, NULL, 0644);

#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
static long als_z3_interim[] =  {0, -5700, 2553000, 953424200};
static long als_z3_interim_motus[] =  {0, -5700, 2553000, 953424200};
static long als_z3[] =          {0, -5700, 2553000, 953424200};
static long als_z3_motus[] =    {0, -5700, 2553000, 953424200};
static long als_z3_zeppelin[] = {-3, -1821, 1404801, 1011675842};
#else // Calgary
static long als_z3[] =  {0, -5700, 2553000, 953424200};
#endif
module_param_array(als_z3, long, NULL, 0644);

#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
static long als_z4_interim[] =  {0, -5700, 2553000, 953424200};
static long als_z4_interim_motus[] =  {0, -5700, 2553000, 953424200};
static long als_z4[] =          {0, -5700, 2553000, 953424200};
static long als_z4_motus[] =    {0, -5700, 2553000, 953424200};
static long als_z4_zeppelin[] = {-3, -1821, 1404801, 1011675842};
#else // Calgary
static long als_z4_calgary[] =  {0, -5700, 2553000, 953424200};
#endif
module_param_array(als_z4, long, NULL, 0644);

static unsigned long als_denom = 10000000;
module_param(als_denom, ulong, 0644);

static unsigned dim_values[] = {0x2E, 0x30, 0x50, 0x50, 0x50};
module_param_array(dim_values, uint, NULL, 0644);

/* Convert slider value into LM3535 register value */
static uint8_t lm3535_convert_value (unsigned value, unsigned zone)
{
    uint8_t reg;
    uint32_t res;

    if (!value)
        return 0;

    if (atomic_read (&lm3535_data.in_suspend)) {
        printk_br ("%s: in TCMD SUSPEND, returning 0x%x\n", 
            __FUNCTION__, value/2);
        return value/2;
    }
    switch (zone) {
        case 0:
            if (value == 1)
                res = dim_values[0];   // DIM value
            else
                res =  als_z0[0] * value * value * value
                      +als_z0[1] * value * value
                      +als_z0[2] * value
                      +als_z0[3];
            break;
        case 1:
            if (value == 1)
                res = dim_values[1];   // DIM value
            else
                res =  als_z1[0] * value * value * value
                      +als_z1[1] * value * value
                      +als_z1[2] * value
                      +als_z1[3];
            break;
        case 2:
            if (value == 1)
                res = dim_values[2];   // DIM value
            else
                res =  als_z2[0] * value * value * value
                      +als_z2[1] * value * value
                      +als_z2[2] * value
                      +als_z2[3];
            break;
        case 3:
            if (value == 1)
                res = dim_values[3];   // DIM value
            else
                res =  als_z3[0] * value * value * value
                      +als_z3[1] * value * value
                      +als_z3[2] * value
                      +als_z3[3];
            break;
        case 4:
        default:
            if (value == 1)
                res = dim_values[4];   // DIM value
            else
                res =  als_z4[0] * value * value * value
                      +als_z4[1] * value * value
                      +als_z4[2] * value
                      +als_z4[3];
            break;
    }
    if (value == 1)
        reg = res;
    else
        reg = res / als_denom;
    printk_br (KERN_INFO "%s: v=%d, z=%d, res=0x%x, reg=0x%x\n", 
        __FUNCTION__, value, zone, res, reg);
    return reg;
}

static void lm3535_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    struct i2c_client *client = lm3535_data.client;
    int ret, nsteps;
    unsigned int total_time;
    unsigned breg = LM3535_BRIGHTNESS_CTRL_REG_A;
    unsigned bright_zone;
    unsigned bvalue;
    unsigned do_ramp = 1;

    printk_br ("%s: %s, 0x%x (%d)\n", __FUNCTION__, 
        led_cdev->name, value, value);
    if (!lm3535_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    if (strstr (led_cdev->name, "nr"))
        do_ramp = 0;

    bright_zone = atomic_read (&lm3535_data.bright_zone);
    mutex_lock (&lm3535_mutex);
    if (value == -1) { // Special case for ALS adjustment
        value = led_cdev->brightness;
    }

#ifdef CONFIG_LM3535_ESD_RECOVERY
    if (value == LED_OFF && esd_polling)
    {
        esd_poll_stop(lm3535_check_esd);
        esd_polling = 0;
    }
#endif /* CONFIG_LM3535_ESD_RECOVERY */
    if (!lm3535_data.enabled && value != 0) {
#ifdef CONFIG_MACH_MOT
        lm3535_enable(client, 1);
#else
        lm3535_enable (client, 1, lm3535_data.button_enabled);
#endif
    }

    /* Calculate brightness value for each zone relative to its cap */
    bvalue = lm3535_convert_value (value, bright_zone);

    /* Calculate number of steps for ramping */
    nsteps = bvalue - lm3535_data.bvalue;
    if (nsteps < 0)
        nsteps = nsteps * (-1);

    lm3535_set_ramp (client, do_ramp, nsteps, &total_time);

    printk_br ("%s: zone %d, 0x%x => 0x%x, %d steps, ramp time %dus\n",
        __FUNCTION__, bright_zone,
        lm3535_data.bvalue, bvalue, nsteps, total_time);

    /* Write to each zone brightness register so that when it jumps into
     * the next zone the value is adjusted automatically 
     */
    ret = lm3535_write_reg (breg, bvalue, __FUNCTION__);
    lm3535_data.bvalue = bvalue;

    if (value == 0) {
        /* Disable everything */
        if (do_ramp) {
            /* Wait for ramping to finish */
            udelay (total_time);
        }
#ifdef CONFIG_MACH_MOT
        lm3535_enable(client, 0);
#else
        lm3535_enable (client, 0, 0);
#endif

    }

#ifdef CONFIG_LM3535_ESD_RECOVERY
    if ((value > 0) && (!esd_polling))
    {
        esd_poll_start(lm3535_check_esd, 0);
        esd_polling = 1;
    }
#endif /* CONFIG_LM3535_ESD_RECOVERY */


    mutex_unlock (&lm3535_mutex);
}

#ifdef CONFIG_LM3535_BUTTON_BL
static void lm3535_button_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    int ret;
    unsigned breg = LM3535_BRIGHTNESS_CTRL_REG_C;
    struct i2c_client *client = lm3535_data.client;

    printk_br ("%s: %s, 0x%x (%d)\n", __FUNCTION__, 
        led_cdev->name, value, value);

    mutex_lock (&lm3535_mutex);

    if (!lm3535_data.button_enabled && value != 0) {
        lm3535_enable (client, lm3535_data.enabled, 1);
    }

    ret = lm3535_write_reg (breg, 0xF8, __FUNCTION__); // Lowest setting

    if (value == 0) {
#ifdef CONFIG_MACH_MOT
    lm3535_enable(client, 0);
#else
    lm3535_enable (client, lm3535_data.enabled, 0);
#endif
    }

    mutex_unlock(&lm3535_mutex);
}
#endif

static int lm3535_als_open (struct inode *inode, struct file *file)
{
    if (!lm3535_data.initialized)
        return -ENODEV;

    return 0;
}

static int lm3535_als_release (struct inode *inode, struct file *file)
{
    return 0;
}
#ifndef CONFIG_MACH_MOT
    int lm3535_disable_esd = 0;
#endif
#define CMD_LEN 5
static ssize_t lm3535_als_write (struct file *fp, const char __user *buf, 
    size_t count, loff_t *pos)
{
    unsigned char cmd[CMD_LEN];
    int len;
    uint8_t value;
    unsigned old_zone;

    if (count < 1)
        return 0;

    len = count > CMD_LEN-1 ? CMD_LEN-1 : count;

    if (copy_from_user (cmd, buf, len))
        return -EFAULT;

    if (lm3535_data.revision <= 1)
        return -EFAULT;

    cmd[len] = '\0';
    if (cmd[len-1] == '\n') {
        cmd[len-1] = '\0';
        len--;
    }
    if (!strcmp (cmd, "1")) {
        printk (KERN_INFO "%s: enabling ALS\n", __FUNCTION__);
        value = CONFIG_VALUE | 0x80;
        mutex_lock (&lm3535_mutex);
        atomic_set (&lm3535_data.use_als, 1);
        /* No need to change ALS zone; interrupt handler will do it */
        lm3535_write_reg (LM3535_CONFIG_REG, value, __FUNCTION__);
        mutex_unlock (&lm3535_mutex);
    } else if (!strcmp (cmd, "0")) {
        printk (KERN_INFO "%s: disabling ALS\n", __FUNCTION__);
        value = CONFIG_VALUE_NO_ALS;
        mutex_lock (&lm3535_mutex);
        old_zone = atomic_read (&lm3535_data.als_zone);
        lm3535_write_reg (LM3535_CONFIG_REG, value, __FUNCTION__);
        atomic_set (&lm3535_data.use_als, 0);
        atomic_set (&lm3535_data.als_zone, ALS_NO_ZONE);
        mutex_unlock (&lm3535_mutex);
        if (atomic_read (&lm3535_data.bright_zone) < 2) {
            atomic_set (&lm3535_data.bright_zone, ALS_NO_ZONE);
            printk_als ("%s: ALS canceled; changing brightness\n",
                __FUNCTION__);
            /* Adjust brightness */
            lm3535_brightness_set (&lm3535_led, -1);
        } else {
            atomic_set (&lm3535_data.bright_zone, ALS_NO_ZONE);
        }
        lm3535_call_als_callbacks (old_zone, 0);
        lm3535_send_als_event (0);
#ifndef CONFIG_MACH_MOT
    } else if (!strcmp (cmd, "e")) {
        lm3535_disable_esd = 1;
    } else if (!strcmp (cmd, "E")) {
        lm3535_disable_esd = 0;
    } else if (!strcmp (cmd, "p")) {
        pwm_value = 0;
    } else if (!strcmp (cmd, "P")) {
        pwm_value = 1;
#endif
    } else {
        printk (KERN_ERR "%s: invalid command %s\n", __FUNCTION__, cmd);
        return -EFAULT;
    }
    
    return count;
}

static ssize_t lm3535_als_read (struct file *file, char __user *buf, 
    size_t count, loff_t *ppos)
{
    char z[20];

    if (file->private_data)
        return 0;

    if (!atomic_read (&lm3535_data.use_als)) {
        sprintf (z, "%d\n", ALS_NO_ZONE);
    } else {
        sprintf (z, "%d %d\n", 
            atomic_read (&lm3535_data.als_zone), 
            atomic_read (&lm3535_data.bright_zone));
    }
    if (copy_to_user (buf, z, strlen (z)))
        return -EFAULT;

    file->private_data = (void *)1;
    return strlen (z);
}

static const struct file_operations als_fops = {
    .owner      = THIS_MODULE,
    .read       = lm3535_als_read,
    .write      = lm3535_als_write,
    .open       = lm3535_als_open,
    .release    = lm3535_als_release,
};

static struct miscdevice als_miscdev = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "als",
    .fops       = &als_fops,
};

static ssize_t lm3535_suspend_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    sprintf (buf, "%d\n", atomic_read (&lm3535_data.in_suspend));
    return strlen(buf)+1;
}

static ssize_t lm3535_suspend_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned value = 0;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }

    sscanf (buf, "%d", &value);
    if (value) {
        printk (KERN_INFO "%s: going into TCMD SUSPEND mode\n",
            __FUNCTION__);
        atomic_set (&lm3535_data.in_suspend, 1);
        lm3535_data.saved_bvalue = lm3535_led.brightness;
        lm3535_led.brightness = 255;
    } else {
        printk (KERN_INFO "%s: exiting TCMD SUSPEND mode\n",
            __FUNCTION__);
        atomic_set (&lm3535_data.in_suspend, 0);
        lm3535_led.brightness = lm3535_data.saved_bvalue;
    }
    /* Adjust brightness */
    lm3535_brightness_set (&lm3535_led, -1);
    return size;
}
static DEVICE_ATTR(suspend, 0644, lm3535_suspend_show, lm3535_suspend_store);

/* This function is called by i2c_probe */
static int lm3535_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int ret = 0;
    unsigned long request_flags =  IRQF_TRIGGER_LOW;
    char *machine;
#ifdef CONFIG_MACH_MOT
    extern unsigned mot_hw_rev;
    unsigned display_id = msm_fb_mddi_nv_get_manprodid();
#else
    unsigned mot_hw_rev = 0;
#endif

#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
    if (machine_is_morrison ()) {
        machine = "morrison";
        memcpy (als_zb, morrison_als_zb, sizeof (als_zb));
        if (display_id == 0) {
            printk (KERN_INFO "%s: interim display\n", __FUNCTION__);
            memcpy (als_z0, als_z0_interim, sizeof (als_z0));
            memcpy (als_z1, als_z1_interim, sizeof (als_z1));
            memcpy (als_z2, als_z2_interim, sizeof (als_z2));
            memcpy (als_z3, als_z3_interim, sizeof (als_z3));
            memcpy (als_z4, als_z4_interim, sizeof (als_z4));
        }
    } else if (machine_is_zeppelin()) { //re-using morrison settings
        memcpy (als_zb, zeppelin_als_zb, sizeof (als_zb));
        memcpy (als_z0, als_z0_zeppelin, sizeof (als_z0));
        memcpy (als_z1, als_z1_zeppelin, sizeof (als_z1));
        memcpy (als_z2, als_z2_zeppelin, sizeof (als_z2));
        memcpy (als_z3, als_z3_zeppelin, sizeof (als_z3));
        memcpy (als_z4, als_z4_zeppelin, sizeof (als_z4));
        machine = "zeppelin";
        resistor_value = 0x10;
    } else if (machine_is_motus ()) {
        memcpy (als_zb, motus_als_zb, sizeof (als_zb));
        if (display_id == 0) {
            printk (KERN_INFO "%s: interim display\n", __FUNCTION__);
            memcpy (als_z0, als_z0_interim_motus, sizeof (als_z0));
            memcpy (als_z1, als_z1_interim_motus, sizeof (als_z1));
            memcpy (als_z2, als_z2_interim_motus, sizeof (als_z2));
            memcpy (als_z3, als_z3_interim_motus, sizeof (als_z3));
            memcpy (als_z4, als_z4_interim_motus, sizeof (als_z4));
        } else {
            memcpy (als_z0, als_z0_motus, sizeof (als_z0));
            memcpy (als_z1, als_z1_motus, sizeof (als_z1));
            memcpy (als_z2, als_z2_motus, sizeof (als_z2));
            memcpy (als_z3, als_z3_motus, sizeof (als_z3));
            memcpy (als_z4, als_z4_motus, sizeof (als_z4));
        }
        machine = "motus";
        resistor_value = 0x20;
    } else {
        printk (KERN_ERR "%s: not morrison or motus or zeppelin; returning\n",
            __FUNCTION__);
        return -EINVAL;
    }
#else // Calgary
#ifdef SUPPORT_LM3535_2ALS
    resistor_value = 0x02;
#endif
    machine = "calgary";
#endif
    printk (KERN_INFO "%s: enter (%s), I2C address = 0x%x, flags = 0x%x\n", 
        __FUNCTION__, machine, client->addr, client->flags);
    printk (KERN_INFO "%s: %s P%x, resistor = 0x%x, zones: 0x%x 0x%x 0x%x 0x%x\n",
        __FUNCTION__, machine, mot_hw_rev, resistor_value, 
        als_zb[0], als_zb[1], als_zb[2], als_zb[3]);

    /* We should be able to read and write byte data */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
            __FUNCTION__);
        return -ENOTSUPP;
    }
    /* Check LM3535 address */
#ifdef LM3535_ADDRESS_HACK
#ifdef CONFIG_MACH_MOT // Morrison, Motus, Zeppelin
    if (client->addr == 0x38) {
        if (i2c_smbus_xfer(client->adapter, client->addr, 0, 0, 0,
                       I2C_SMBUS_QUICK, NULL) < 0) {
            printk (KERN_ERR "%s: unable to read address 0x38; switching to 0x36\n", 
                __FUNCTION__);
            client->addr = 0x36;
        }
    }
#else // Calgary
    if (client->addr == LM3535_REV6_I2C_ADDR) {
    if (i2c_smbus_xfer(client->adapter, client->addr, 0, 0, 0,
            I2C_SMBUS_QUICK, NULL) < 0) {
        printk(KERN_ERR "%s: unable to read address %x;"
            "switching to %x\n", __func__,
        LM3535_REV6_I2C_ADDR, LM3535_I2C_ADDR);

        client->addr = LM3535_I2C_ADDR;
        }
    }
#endif
#endif

    lm3535_data.client = client;
    i2c_set_clientdata (client, &lm3535_data);

    /* Initialize chip */
    lm3535_setup (lm3535_data.client);

    /* Initialize interrupts */
    if (lm3535_data.revision > 1) {
        INIT_WORK(&lm3535_data.work, lm3535_work_func);
        if (client->irq) {
            ret = request_irq (client->irq, lm3535_irq_handler, request_flags, 
                "lm3535", &lm3535_data);

            if (ret == 0) {
                lm3535_data.use_irq = 1;
                ret = set_irq_wake (client->irq, 1);
            } else {
                printk (KERN_ERR "request_irq %d for lm3535 failed: %d\n", 
                    client->irq, ret);
                free_irq (client->irq, &lm3535_data);
                lm3535_data.use_irq = 0;
            }
        }
    }

    /* Register LED class */
    ret = led_classdev_register (&client->adapter->dev, &lm3535_led);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, lm3535_led.name, ret);
        return ret;
    } 

    /* Register LED class for no ramping */
#ifdef CONFIG_MACH_MOT
    ret = led_classdev_register (&client->adapter->dev, &lm3535_led_noramp);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, lm3535_led.name, ret);
    } 
#endif
    if ((ret = misc_register (&als_miscdev))) {
        printk (KERN_ERR "%s: misc_register failed, error %d\n",
            __FUNCTION__, ret);
        led_classdev_unregister (&lm3535_led);
#ifdef CONFIG_MACH_MOT
    led_classdev_unregister(&lm3535_led_noramp);
#endif
        return ret;
    }

#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
    /* Register LED class for button leds */
    ret = led_classdev_register (&client->adapter->dev, &lm3535_led_button);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, lm3535_led.name, ret);
        misc_deregister (&als_miscdev);
        led_classdev_unregister (&lm3535_led);
        return ret;
    } 
#endif
#endif

    atomic_set (&lm3535_data.in_suspend, 0);
    ret = device_create_file (lm3535_led.dev, &dev_attr_suspend);
    if (ret) {
      printk (KERN_ERR "%s: unable to create suspend device file for %s: %d\n",
            __FUNCTION__, lm3535_led.name, ret);
        led_classdev_unregister (&lm3535_led);
#ifdef CONFIG_MACH_MOT
        led_classdev_unregister(&lm3535_led_noramp);
#else
        led_classdev_unregister (&lm3535_led_button);
#endif
        misc_deregister (&als_miscdev);
        return ret;
    }
    dev_set_drvdata (lm3535_led.dev, &lm3535_led);
    lm3535_data.idev = input_allocate_device();
    if (lm3535_data.idev == NULL) {
      printk (KERN_ERR "%s: unable to allocate input device file for als\n",
            __FUNCTION__);
        led_classdev_unregister (&lm3535_led);
#ifdef CONFIG_MACH_MOT
        led_classdev_unregister(&lm3535_led_noramp);
#else
        led_classdev_unregister (&lm3535_led_button);
#endif
        misc_deregister (&als_miscdev);
        device_remove_file (lm3535_led.dev, &dev_attr_suspend);
        return -ENOMEM;
    }
	lm3535_data.idev->name = "als";
	input_set_capability(lm3535_data.idev, EV_MSC, MSC_RAW);
	input_set_capability(lm3535_data.idev, EV_LED, LED_MISC);
    ret = input_register_device (lm3535_data.idev);
    if (ret) {
      printk (KERN_ERR "%s: unable to register input device file for als: %d\n",
            __FUNCTION__, ret);
        led_classdev_unregister (&lm3535_led);
#ifdef CONFIG_MACH_MOT
        led_classdev_unregister(&lm3535_led_noramp);
#else
        led_classdev_unregister (&lm3535_led_button);
#endif
        misc_deregister (&als_miscdev);
        device_remove_file (lm3535_led.dev, &dev_attr_suspend);
        input_free_device (lm3535_data.idev);
        return ret;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend (&early_suspend_data);
#endif

    lm3535_led.brightness = 255;
    lm3535_led_noramp.brightness = 255;
    //lm3535_brightness_set (&lm3535_led_noramp, 255);
    lm3535_write_reg (LM3535_BRIGHTNESS_CTRL_REG_A, 0x79, __FUNCTION__);
    lm3535_data.initialized = 1;

    return 0;
}

static irqreturn_t lm3535_irq_handler (int irq, void *dev_id)
{
    struct lm3535 *data_ptr = (struct lm3535 *)dev_id;

    pr_debug ("%s: got an interrupt %d\n", __FUNCTION__, irq);

    disable_irq (irq);
    schedule_work (&data_ptr->work);

    return IRQ_HANDLED;
}

static void lm3535_send_als_event (int zone)
{
#ifdef CONFIG_ALS_UEVENT
    char event_string[20];
    char *envp[] = {event_string, NULL};
    int ret;

    sprintf (event_string, "ALS_ZONE=%d", zone);
    ret = kobject_uevent_env (&als_miscdev.this_device->kobj,
        KOBJ_CHANGE, envp);
    if (ret) {
        printk (KERN_ERR "%s: kobject_uevent_env failed: %d\n",
            __FUNCTION__, ret);
    } else {
        printk_event ("%s: kobject_uevent_env %s success\n",
            __FUNCTION__, event_string);
    }
#else
    //input_event (lm3535_data.idev, EV_MSC, MSC_RAW, light_value);
	input_event (lm3535_data.idev, EV_LED, LED_MISC, zone);
	input_sync (lm3535_data.idev);

#endif
}

static void lm3535_call_als_callbacks(unsigned old_zone, unsigned zone)
{
    struct als_callback *c;
    unsigned old, new;

    old = (old_zone == ALS_NO_ZONE) ? 0 : old_zone;
    new = (zone == ALS_NO_ZONE) ? 0 : zone;

    mutex_lock (&als_cb_mutex);
    list_for_each_entry(c, &als_callbacks, entry) {
#ifdef CONFIG_MACH_MOT
    c->cb(old, new, c->cookie);
#else
    c->cb(old, new);
#endif
    }
    mutex_unlock (&als_cb_mutex);
}

static void lm3535_work_func (struct work_struct *work)
{
    int ret;
    uint8_t reg;
    unsigned zone, old_zone;

    pr_debug ("%s: work function called\n", __FUNCTION__);
    ret = lm3535_read_reg (LM3535_ALS_REG, &reg);
    if (ret) {
        if (reg & ALS_FLAG_MASK) {
            zone = reg & ALS_ZONE_MASK;
            if (zone > 4)
                zone = 4;
            old_zone = atomic_read (&lm3535_data.als_zone);
            printk_als ("%s: ALS zone changed: %d => %d, register = 0x%x\n",
                __FUNCTION__, old_zone, zone, reg);
            atomic_set (&lm3535_data.als_zone, zone);
            if (zone > atomic_read (&lm3535_data.bright_zone) ||
                atomic_read (&lm3535_data.bright_zone) == ALS_NO_ZONE) {
                atomic_set (&lm3535_data.bright_zone, zone);
                if (!atomic_read (&lm3535_data.in_suspend)) {
                    printk_als ("%s: ALS zone increased; changing brightness\n",
                        __FUNCTION__);
                    /* Adjust brightness */
                    lm3535_brightness_set (&lm3535_led, -1);
                } else {
                    printk_als ("%s: ALS zone increased; SUSPEND mode - not changing brightness\n",
                        __FUNCTION__);
                }
            }
            /* See if PWM needs to be changed */
            if (old_zone < 2 && zone >= 2) {
                lm3535_write_reg (LM3535_CONFIG_REG, CONFIG_VALUE | 0x80, 
                    __FUNCTION__);
                printk_als ("%s: moved from dim/dark to bright; disable PWM\n",
                    __FUNCTION__);
            } else if (old_zone >= 2 && zone < 2) {
                lm3535_write_reg (LM3535_CONFIG_REG, 
                    CONFIG_VALUE|0x80|pwm_value, 
                    __FUNCTION__);
                printk_als ("%s: moved from bright to dim/dark; enable PWM\n",
                    __FUNCTION__);
            }
            if (!atomic_read (&lm3535_data.in_suspend)) {
                lm3535_call_als_callbacks (old_zone, zone); 
            }
            lm3535_send_als_event (zone);
        } else {
            printk_als ("%s: got ALS interrupt but flag is not set: 0x%x\n", 
                __FUNCTION__, reg);
        }
    }
    if (atomic_read (&lm3535_data.do_als_config)) {
        lm3535_write_reg (LM3535_ALS_CTRL_REG, ALS_AVERAGING, __FUNCTION__);
        atomic_set (&lm3535_data.do_als_config, 0);
        printk_als ("%s: configured ALS averaging 0x%x\n",
            __FUNCTION__, ALS_AVERAGING);
    }
    enable_irq (lm3535_data.client->irq);
}

#if 0
static enum hrtimer_restart lm3535_timer_func (struct hrtimer *timer)
{
    schedule_work(&lm3535_data.work);

    hrtimer_start(&lm3535_data.timer, 
        ktime_set(1, 0), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}
#endif

static void lm3535_set_options_r1 (uint8_t *buf, unsigned ramp)
{
    struct lm3535_options_register_r1 *r =
        (struct lm3535_options_register_r1 *)buf;

    if (!r)
        return;
    *buf = 0;
    r->rs = ramp;
    r->gt = 0x2;
    r->rev = 0;
}

static void lm3535_set_options_r2 (uint8_t *buf, unsigned ramp)
{
    struct lm3535_options_register_r2 *r =
        (struct lm3535_options_register_r2 *)buf;

    if (!r)
        return;
    *buf = 0;
    r->rs_up = ramp;
    r->rs_down = ramp;
    r->gt = 0;
}

static void lm3535_set_options_r3 (uint8_t *buf, unsigned ramp)
{
    struct lm3535_options_register_r3 *r =
        (struct lm3535_options_register_r3 *)buf;

    if (!r)
        return;
    *buf = 0;
    r->rs_up = ramp;
    r->rs_down = ramp;
    r->gt = 0x02;
}

/* This function calculates ramp step time so that total ramp time is
 * equal to ramp_time defined currently at 200ms
 */
static int lm3535_set_ramp (struct i2c_client *client, 
    unsigned int on, unsigned int nsteps, unsigned int *rtime)
{
    int ret, i = 0;
    uint8_t value = 0;
    unsigned int total_time = 0;

    if (on) {
        /* Calculate the closest possible ramp time */
        for (i = 0; i < lm3535_data.nramp; i++) {
            total_time = nsteps * lm3535_ramp[i];
            if (total_time >= ramp_time)
                break;
        }
        if (i > 0 && total_time > ramp_time) {
#if 0
            /* If previous value is closer */
            if (total_time - ramp_time > 
                    ramp_time - nsteps * lm3535_ramp[i-1]) {
                i--;
                total_time = nsteps * lm3535_ramp[i];
            }
#endif
            i--;
            total_time = nsteps * lm3535_ramp[i];
        }
        lm3535_set_options_f (&value, i);
    } 
     
#if 0
    printk (KERN_ERR "%s: ramp = %s, ramp step = %d us (total = %d us)\n",
        __FUNCTION__, on ? "on" : "off", lm3535_ramp[i], total_time);
#endif
    if (rtime)
        *rtime = total_time;
    ret = lm3535_write_reg (LM3535_OPTIONS_REG, value, __FUNCTION__);
#if 0
    printk_br ("%s: nsteps = %d, OPTIONS_REG = 0x%x, total ramp = %dus\n",
        __FUNCTION__, nsteps, value, lm3535_ramp[i] * nsteps);
#endif
    return ret;
}
#ifdef CONFIG_MACH_MOT
static int lm3535_enable(struct i2c_client *client, unsigned int on)

#else
static int lm3535_enable (struct i2c_client *client, unsigned int on, unsigned int button_on)
#endif
{
    int ret;
    uint8_t value = 0x3F; // Enable A, D53, D62

    if (!on)
        value = 0;

#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
    if (button_on) 
        value |= 0x80; // Enable D1C
#endif
#endif

    ret = lm3535_write_reg (LM3535_ENABLE_REG, value, __FUNCTION__);
    if (ret < 0) 
        return ret;

    if (lm3535_data.revision == 2 || lm3535_data.revision == 3) {
        if (on) {
            ret = lm3535_write_reg (LM3535_TRIM_REG, LM3535_TRIM_VALUE, 
                __FUNCTION__);
        }
    }
    lm3535_data.enabled = on;
#ifndef CONFIG_MACH_MOT
    lm3535_data.button_enabled = button_on;
#endif
    return ret;
}

static int lm3535_configure (void)
{
    int ret = 0;
    uint8_t value = 0x03;
    uint8_t reg = 0;
    unsigned old_zone = 0;
    unsigned new_zone = ALS_NO_ZONE;

    /* Config register bits:
     * Rev1: AVE2  AVE1    AVE0     ALS-SD   ALS-EN PWM-EN  53A    62A
     * Rev2: ALSF  ALS-SD  ALS-ENB  ALS-ENA  62A    53A     PWM-P  PWM-EN
     * Rev3: ALSF  ALS-EN  ALS-ENB  ALS-ENA  62A    53A     PWM-P  PWM-EN
     * 
     * ALSF: sets E1B/INT pin to interrupt Pin; 0 = D1B, 1 = INT.
     *     Open Drain Interrupt.  Pulls low when change occurs.  Flag cleared
     *     once a I2C read command for register 0x40 occurs.
     * ALS-SD (rev2) and ALS-EN (final):  turn off ALS feature
     *     ALS-SD: 0 = Active, 1 = Shutdown. Rev2 cannot have ALS active without
     *         LEDs turned on (ENxx bits = 1).
     *     ALS-EN: 0 = Shutdown, 1 = Active. Final can have ALS active without
     *         LEDs turned on.  ALS-EN overrides the ENC bit.
     * ALS-ENB and ALS-ENA: 1 enables ALS control of diode current. BankA has
     *     full ALS control, BankB just has on/off ability.
     * 62A and 53A: 1 sets D62 and D53 to BankA (Required for 6 LEDs in BankA).
     *     0 sets them to BankB.
     * PWM-P: PWM Polarity, 0 = Active (Diodes on) High, 1 = Active (Diodes on)
     *     Low.
     * PWM-EN: Enables PWM Functionality. 1 = Active.
     */

#ifndef CONFIG_MAC_MOT
#ifdef CONFIG_LM3535_ESD_RECOVERY
    /* Configure lighting zone max brightness */
    lm3535_write_reg (LM3535_ALS_Z0T_REG, als_zone_max[0],
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z1T_REG, als_zone_max[1],
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z2T_REG, als_zone_max[2],
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z3T_REG, als_zone_max[3],
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z4T_REG, als_zone_max[4],
        __FUNCTION__);
#endif
#endif

    if (lm3535_data.revision > 1) {
        /* Configure internal ALS resistor register */
        ret = lm3535_write_reg (LM3535_ALS_RESISTOR_REG, 
            (uint8_t)resistor_value, __FUNCTION__);

#ifndef CONFIG_MAC_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
        ret = lm3535_write_reg (LM3535_ALS_SELECT_REG, 0x2, 
            __FUNCTION__);
#endif
#endif
        /* Configure lighting zone boundaries */
        lm3535_write_reg (LM3535_ALS_ZB0_REG, als_zb[0], __FUNCTION__);
        lm3535_write_reg (LM3535_ALS_ZB1_REG, als_zb[1], __FUNCTION__);
        lm3535_write_reg (LM3535_ALS_ZB2_REG, als_zb[2], __FUNCTION__);
        lm3535_write_reg (LM3535_ALS_ZB3_REG, als_zb[3], __FUNCTION__);

        /* Configure ALS averaging to be very short the first time */
        lm3535_write_reg (LM3535_ALS_CTRL_REG, 0x80, __FUNCTION__);
    }

    if (lm3535_data.revision == 0) {
        value = 0x3;  // Just enable A, don't bother with ALS
        atomic_set (&lm3535_data.use_als, 0);
    } else if (lm3535_data.revision == 1) {
        value = 0x0C; // No ALS or PWM
        atomic_set (&lm3535_data.use_als, 0);
    } else {
        if (atomic_read (&lm3535_data.use_als))
            value = CONFIG_VALUE;
        else
            value = CONFIG_VALUE_NO_ALS;
    }
    pr_debug ("%s: use_als is %d, value = 0x%x\n", __FUNCTION__,
        atomic_read (&lm3535_data.use_als), value);
    ret = lm3535_write_reg (LM3535_CONFIG_REG, value, __FUNCTION__);
    // Nothing else to do for older revisions
    if (lm3535_data.revision <= 1) {
        return 0;
    }

    /* Has to be at least 300ms even with ALS averaging set to 0 */
    msleep_interruptible (als_sleep);  // Wait for ALS to kick in
    /* Read current ALS zone */
    old_zone = atomic_read (&lm3535_data.als_zone);
    if (atomic_read (&lm3535_data.use_als)) {
        ret = lm3535_read_reg (LM3535_ALS_REG, &reg);
        if (ret) {
            new_zone = reg & ALS_ZONE_MASK;
            if (new_zone > 4) {
                new_zone = 4;
            }
            atomic_set (&lm3535_data.als_zone, new_zone);
            printk_als ("%s: ALS Register: 0x%x, zone %d\n",
                __FUNCTION__, reg, atomic_read (&lm3535_data.als_zone));
        } else {
            atomic_set (&lm3535_data.als_zone, ALS_NO_ZONE);
            atomic_set (&lm3535_data.use_als, 0);
            printk (KERN_ERR "%s: unable to read ALS zone; disabling ALS\n",
                __FUNCTION__);
        }
    } else {
        atomic_set (&lm3535_data.als_zone, ALS_NO_ZONE);
    }
    /* Brightness zone is for now the same as ALS zone */
    atomic_set (&lm3535_data.bright_zone,
        atomic_read (&lm3535_data.als_zone));
    lm3535_call_als_callbacks (old_zone, new_zone);
    if (lm3535_data.initialized) {
        lm3535_send_als_event (new_zone);
    }
    if (atomic_read (&lm3535_data.use_als)) {
        /* Configure averaging */
        //lm3535_write_reg (LM3535_ALS_CTRL_REG, ALS_AVERAGING, __FUNCTION__);
        /* Enable interrupt and PWM for CABC */
        if (new_zone <= 1) {
            lm3535_write_reg (LM3535_CONFIG_REG, CONFIG_VALUE|0x80|pwm_value, 
                __FUNCTION__);
        } else {
            lm3535_write_reg (LM3535_CONFIG_REG, CONFIG_VALUE | 0x80, 
                __FUNCTION__);
        }
    } else {
        lm3535_write_reg (LM3535_CONFIG_REG, CONFIG_VALUE | pwm_value, 
            __FUNCTION__);
    }

    return ret;
}

static int lm3535_setup (struct i2c_client *client)
{
    int ret;
    uint8_t value;

    /* Read revision number */
    ret = lm3535_read_reg (LM3535_ALS_CTRL_REG, &value);
    if (ret < 0) {
        printk (KERN_ERR "%s: unable to read from chip: %d\n",
            __FUNCTION__, ret);
        return ret;
    }
    switch (value) {
        case 0xFF: lm3535_data.revision = 0; break;
        case 0xF0: lm3535_data.revision = 1; break;
#ifndef CONFIG_MACH_MOT
#ifdef SUPPORT_LM3535_2ALS
        case 0x02: lm3535_data.revision = 5; break;
#else
        case 0x02: lm3535_data.revision = 2; break;
#endif
#else
        case 0x02: lm3535_data.revision = 2; break;
#endif
        case 0x00: lm3535_data.revision = 3; break;
        case 0x01: lm3535_data.revision = 4; break;
        default: lm3535_data.revision = 4; break; // Assume final
    }
    /* revision is going to be an index to lm3535_ramp array */
    printk (KERN_INFO "%s: revision %d (0x%X)\n", 
        __FUNCTION__, lm3535_data.revision+1, value);
    if (lm3535_data.revision == 0) {
        lm3535_ramp = lm3535_ramp_r1;
        lm3535_set_options_f = lm3535_set_options_r1;
    } else if (lm3535_data.revision == 1) {
        lm3535_ramp = lm3535_ramp_r2;
        lm3535_set_options_f = lm3535_set_options_r2;
    } else {
        lm3535_ramp = lm3535_ramp_r3;
        lm3535_set_options_f = lm3535_set_options_r3;
        lm3535_data.nramp = 8;
    }

#ifndef CONFIG_MACH_MOT
#ifndef CONFIG_LM3535_ESD_RECOVERY
    /* Configure lighting zone max brightness */
    lm3535_write_reg (LM3535_ALS_Z0T_REG, als_zone_max[0],
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z1T_REG, als_zone_max[1], 
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z2T_REG, als_zone_max[2], 
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z3T_REG, als_zone_max[3], 
        __FUNCTION__);
    lm3535_write_reg (LM3535_ALS_Z4T_REG, als_zone_max[4], 
        __FUNCTION__);
#endif /* CONFIG_LM3535_ESD_RECOVERY */
#endif
    /* PWM */
    if (lm3535_data.revision < 4) {
        pwm_value = 0;
    }
    atomic_set (&lm3535_data.als_zone, ALS_NO_ZONE);
    atomic_set (&lm3535_data.use_als, 1);
    atomic_set (&lm3535_data.do_als_config, 1);
    ret = lm3535_configure ();
    if (ret < 0)
        return ret;
#ifdef CONFIG_MACH_MOT
    ret = lm3535_enable(client, 1);
#else
    ret = lm3535_enable(client, 1, 1);
#endif
    if (ret < 0)
        return ret;

    //hrtimer_init (&lm3535_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //lm3535_data.timer.function = lm3535_timer_func;
    //hrtimer_start(&lm3535_data.timer, ktime_set(2, 0), HRTIMER_MODE_REL);

    return ret;
}

static int lm3535_remove (struct i2c_client *client)
{
    struct lm3535 *data_ptr = i2c_get_clientdata(client);
    if (data_ptr->use_irq)
        free_irq (client->irq, data_ptr);
    led_classdev_unregister (&lm3535_led);
#ifdef CONFIG_MACH_MOT
    led_classdev_unregister(&lm3535_led_noramp);
#endif
/*    led_classdev_unregister (&lm3535_led_noramp); */
    misc_deregister (&als_miscdev);
    device_remove_file (lm3535_led.dev, &dev_attr_suspend);

#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
    led_classdev_unregister (&lm3535_led_button);
#endif
#endif
    input_unregister_device (lm3535_data.idev);
    input_free_device (lm3535_data.idev);
    return 0;
}

static int lm3535_suspend (struct i2c_client *client, pm_message_t mesg)
{
    printk_suspend ("%s: called with pm message %d\n", 
        __FUNCTION__, mesg.event);

#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_ESD_RECOVERY
    if (esd_polling)
        esd_poll_stop(lm3535_check_esd);
#endif
#endif
     
    led_classdev_suspend (&lm3535_led);
#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
    led_classdev_suspend (&lm3535_led_button);
#endif
#endif
    /* Disable ALS interrupt */
    lm3535_write_reg (LM3535_CONFIG_REG, 0, __FUNCTION__);
    /* Put ALS Resistor into high impedance mode to save current */
    lm3535_write_reg (LM3535_ALS_RESISTOR_REG, 0, __FUNCTION__);

    /* Reset ALS averaging */
    lm3535_write_reg (LM3535_ALS_CTRL_REG, 0, __FUNCTION__);
    atomic_set (&lm3535_data.do_als_config, 1);
    return 0;
}

static int lm3535_resume (struct i2c_client *client)
{
    printk_suspend ("%s: resuming\n", __FUNCTION__);
    mutex_lock (&lm3535_mutex);
    lm3535_configure ();
    mutex_unlock (&lm3535_mutex);
    led_classdev_resume (&lm3535_led);
#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
    led_classdev_resume (&lm3535_led_button);
#endif
#ifdef CONFIG_LM3535_ESD_RECOVERY
    /* No need to restart esd polling here,
        led_classdev_resume() will invoke brightness set on it's own */
#endif
#else
    msm_fb_led_resumed();
#endif
    printk_suspend ("%s: driver resumed\n", __FUNCTION__);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3535_early_suspend (struct early_suspend *h)
{
    lm3535_suspend (lm3535_data.client, PMSG_SUSPEND);
}

static void lm3535_late_resume (struct early_suspend *h)
{
    lm3535_resume (lm3535_data.client);
}
#endif


static int __devinit lm3535_init (void)
{
    int ret;

    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    ret = i2c_add_driver (&lm3535_driver);
    if (ret) {
        printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n", 
            __FUNCTION__, ret);
    }

    return ret;
}

static void __exit lm3535_exit(void)
{
    i2c_del_driver (&lm3535_driver);
}

static char *reg_name (int reg) 
{
    switch (reg) {
        case (LM3535_ENABLE_REG): return "ENABLE"; break;
        case (LM3535_CONFIG_REG): return "CONFIG"; break;
        case (LM3535_OPTIONS_REG): return "OPTIONS"; break;
        case (LM3535_ALS_REG): return "ALS"; break;
        case (LM3535_ALS_RESISTOR_REG): return "ALS_RESISTOR"; break;
#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_BUTTON_BL
        case (LM3535_ALS_SELECT_REG): return "ALS_SELECT"; break;
#endif
#endif
        case (LM3535_ALS_CTRL_REG): return "ALS CONTROL"; break;
        case (LM3535_BRIGHTNESS_CTRL_REG_A): return "BRIGHTNESS_CTRL_A"; break;
        case (LM3535_BRIGHTNESS_CTRL_REG_B): return "BRIGHTNESS_CTRL_B"; break;
        case (LM3535_BRIGHTNESS_CTRL_REG_C): return "BRIGHTNESS_CTRL_C"; break;
        case (LM3535_ALS_ZB0_REG): return "ALS_ZB0"; break;
        case (LM3535_ALS_ZB1_REG): return "ALS_ZB1"; break;
        case (LM3535_ALS_ZB2_REG): return "ALS_ZB2"; break;
        case (LM3535_ALS_ZB3_REG): return "ALS_ZB3"; break;
        case (LM3535_ALS_Z0T_REG): return "ALS_Z0T"; break;
        case (LM3535_ALS_Z1T_REG): return "ALS_Z1T"; break;
        case (LM3535_ALS_Z2T_REG): return "ALS_Z2T"; break;
        case (LM3535_ALS_Z3T_REG): return "ALS_Z3T"; break;
        case (LM3535_ALS_Z4T_REG): return "ALS_Z4T"; break;
        case (LM3535_TRIM_REG): return "TRIM"; break;
        default: return "UNKNOWN"; break;
    }
    return "UNKNOWN";
}

unsigned lmxxxx_detect_esd (void)
{
    uint8_t value = 0;

    if (!lm3535_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return 0;
    }
    //0 - no ESD, 1 - ESD
    lm3535_read_reg (LM3535_ALS_RESISTOR_REG, &value);
    if (value != (uint8_t)resistor_value)
        return 1;
    return 0;
}
EXPORT_SYMBOL(lmxxxx_detect_esd);

void lmxxxx_fix_esd (void)
{
    if (!lm3535_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    mutex_lock (&lm3535_mutex);
    lm3535_configure ();
    mutex_unlock (&lm3535_mutex);
    return;
}
EXPORT_SYMBOL(lmxxxx_fix_esd);

#ifndef CONFIG_MACH_MOT
#ifdef CONFIG_LM3535_ESD_RECOVERY
void lm3535_check_esd (void* arg)
{
    if (lm3535_disable_esd)
        return;

    if (lmxxxx_detect_esd()) {
        lmxxxx_fix_esd();
        lm3535_enable (lm3535_data.client, lm3535_data.enabled, lm3535_data.button_enabled);
    }
}
EXPORT_SYMBOL(lm3535_check_esd);
#endif
#endif

void lmxxxx_set_pwm (unsigned en_dis)
{
    if (en_dis) {
        pwm_value = 1;
    } else {
        pwm_value = 0;
    }
    printk (KERN_INFO "%s: setting PWM to %d\n", 
        __FUNCTION__, pwm_value);
}
EXPORT_SYMBOL(lmxxxx_set_pwm);
module_init(lm3535_init);
module_exit(lm3535_exit);

MODULE_DESCRIPTION("LM3535 DISPLAY BACKLIGHT DRIVER");
MODULE_LICENSE("GPL v2");
