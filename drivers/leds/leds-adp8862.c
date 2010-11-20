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

// Linux  driver for    ADP8862 display backlight

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

#define MODULE_NAME "leds_adp8862"

/******************************************************************************
 *  ADP8862 registers
 ******************************************************************************/
#define ADP8862_SLAVE_ADDR 0x2A
#define ADP8862_MFDVID_REG 0x00
#define ADP8862_MDCR_REG   0x01
#define ADP8862_MDCR2_REG  0x02
#define ADP8862_INTREN_REG 0x03
#define ADP8862_CFGR_REG   0x04
#define ADP8862_BLSEN_REG  0x05
#define ADP8862_BLOFF_REG  0x06
#define ADP8862_BLDIM_REG  0x07
#define ADP8862_BLFR_REG   0x08
#define ADP8862_BLMX1_REG  0x09  // Daylight max current
#define ADP8862_BLDM1_REG  0x0A  // Daylight dim current
#define ADP8862_BLMX2_REG  0x0B  // Office max current
#define ADP8862_BLDM2_REG  0x0C  // Office dim current
#define ADP8862_BLMX3_REG  0x0D  // Dark max current
#define ADP8862_BLDM3_REG  0x0E  // Dark dim current

#define ADP8862_LED_MAX 0x7F  // Max brightness value supported by ADP8862

#ifdef ALS_NO_ZONE
#undef ALS_NO_ZONE
#endif
#define ALS_NO_ZONE 0 // Daylight zone if not configured

static unsigned als_zone_max[] = {0x7F, 0x65, 0x45};

static unsigned do_trace = 0;
module_param(do_trace, uint, 0644);

#define adpinfo(fmt,args...) if (do_trace) printk(KERN_INFO fmt, ##args)

/* ALS callbacks */
static DEFINE_MUTEX(als_cb_mutex);
static LIST_HEAD(als_callbacks);
struct als_callback {
    als_cb cb;
    uint32_t cookie;
    struct list_head entry;
};

static void adp8862_send_als_event (int zone);
static char *reg_name (int reg);
static int adp8862_configure (void);
static int adp8862_write_reg (unsigned reg, uint8_t value, const char *caller);
static int adp8862_read_reg (unsigned reg, uint8_t *value);
/*static int adp8862_set_ramp (struct i2c_client *client,
    unsigned int on, unsigned int nsteps, unsigned int *rtime);*/
static int adp8862_enable (struct i2c_client *client, unsigned int on);
static int adp8862_probe(struct i2c_client *client,
    const struct i2c_device_id *id);
static int adp8862_setup (struct i2c_client *client);
static int adp8862_remove (struct i2c_client *client);
static void adp8862_work_func (struct work_struct *work);
static irqreturn_t adp8862_irq_handler (int irq, void *dev_id);
static void adp8862_brightness_set(struct led_classdev *led_cdev,
                enum led_brightness value);
//static unsigned adp8862_read_als_zone (void);
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp8862_early_suspend (struct early_suspend *h);
static void adp8862_late_resume (struct early_suspend *h);
static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
    .suspend = adp8862_early_suspend,
    .resume = adp8862_late_resume,
};

#endif
static int adp8862_suspend (struct i2c_client *client, pm_message_t mesg);
static int adp8862_resume (struct i2c_client *client);
#endif

/* LED class struct */
static struct led_classdev adp8862_led = {
    .name = "lcd-backlight",
    .brightness_set = adp8862_brightness_set,
};

/* LED class struct for no ramping */
static struct led_classdev adp8862_led_noramp = {
    .name = "lcd-nr-backlight",
    .brightness_set = adp8862_brightness_set,
};

static const struct i2c_device_id adp8862_id[] = {
    { "adp8862", 0 },
    { }
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver adp8862_driver =
{
    .driver = {
        .name   = "adp8862",
    },
    .id_table = adp8862_id,
    .probe = adp8862_probe,
    .remove  = adp8862_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = adp8862_suspend,
    .resume     = adp8862_resume,
#endif
};

#define ADP8862_NUM_ZONES 3
struct adp8862 {
    uint16_t addr;
    struct i2c_client *client;
    unsigned initialized;
    unsigned enabled;
    int use_irq;
    int revision;
    int nramp;
    atomic_t als_zone;             // Current ALS zone
    atomic_t use_als;              // Whether to use ALS
    unsigned zone_reg[ADP8862_NUM_ZONES];   // Current zone brightness register
    unsigned bvalue[ADP8862_NUM_ZONES];// Current brightness for each zone
    //struct hrtimer timer;
    struct work_struct  work;
};
static DEFINE_MUTEX(adp8862_mutex);

static struct adp8862 adp8862_data = {
    .zone_reg = {ADP8862_BLMX1_REG, ADP8862_BLMX2_REG, ADP8862_BLMX3_REG},
    .bvalue = {0x7F, 0x65, 0x45}
};

int adp8862_register_als_callback (als_cb func, uint32_t cookie)
{
    struct als_callback *c;

    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    c = kzalloc (sizeof (struct als_callback), GFP_KERNEL);
    if (c == NULL) {
        printk (KERN_ERR "%s: unable to register ALS callback: kzalloc\n",
            __FUNCTION__);
        return -ENOMEM;
    }
    c->cb = func;
    c->cookie = cookie;
    mutex_lock (&als_cb_mutex);
    list_add (&c->entry, &als_callbacks);
    mutex_unlock (&als_cb_mutex);
    return 0;
}
EXPORT_SYMBOL(adp8862_register_als_callback);

void adp8862_unregister_als_callback (als_cb func)
{
    struct als_callback *c;

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
    printk (KERN_ERR "%s: callback 0x%x not found\n", __FUNCTION__, (unsigned int)func);
}
EXPORT_SYMBOL(adp8862_unregister_als_callback);

unsigned adp8862_als_is_dark (void)
{
    unsigned zone;

    zone = atomic_read (&adp8862_data.als_zone);
    printk (KERN_ERR "%s: enter, zone = %d\n",
        __FUNCTION__, zone);
    if (zone == 3)
        return 1;
    else
        return 0;
}
EXPORT_SYMBOL(adp8862_als_is_dark);

static int adp8862_read_reg (unsigned reg, uint8_t *value)
{
    struct i2c_client *client = adp8862_data.client;
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

static int adp8862_write_reg (unsigned reg, uint8_t value, const char *caller)
{
    uint8_t buf[2] = {reg, value};
    int ret = 0;

    adpinfo ("%s: writing 0x%X to reg 0x%X (%s) at addr 0x%X\n",
        caller, buf[1], buf[0], reg_name (reg), adp8862_data.client->addr);
    ret = i2c_master_send (adp8862_data.client, buf, 2);
    if (ret < 0)
        printk (KERN_ERR "%s: i2c_master_send error %d\n",
            caller, ret);
    return ret;
}

static void adp8862_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    struct i2c_client *client = adp8862_data.client;
    int ret, nsteps;
    unsigned int total_time = 1;
    unsigned breg = ADP8862_BLMX1_REG;
    unsigned als_zone;
    unsigned bvalue[ADP8862_NUM_ZONES];
    int i;
    unsigned do_ramp = 1;

    printk (KERN_ERR "%s: %s, 0x%x (%d)\n", __FUNCTION__,
        led_cdev->name, value, value);
    if (!adp8862_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    if (strstr (led_cdev->name, "nr"))
        do_ramp = 0;

    if (value > LED_FULL) // Value can't be greater than 255
        value = LED_FULL;

    als_zone = atomic_read (&adp8862_data.als_zone);
    mutex_lock (&adp8862_mutex);

    //als_zone = adp8862_read_als_zone ();

    if (!adp8862_data.enabled && value != 0) {
        adp8862_enable (client, 1);
    }

    /* Calculate brightness value for each zone relative to its cap */
    value = value / 2;  // Convert into ADP8862 scale
    for (i = 0; i < ADP8862_NUM_ZONES; i++) {
        bvalue[i] = value * als_zone_max[i] / ADP8862_LED_MAX;
    }

    /* Brightness register for the current zone */
    breg = adp8862_data.zone_reg[als_zone];

    /* Calculate number of steps for ramping */
    nsteps = adp8862_data.bvalue[als_zone] - bvalue[als_zone];
    if (nsteps < 0)
        nsteps = nsteps * (-1);

    adpinfo ("%s: zone %d, brightness 0x%x => 0x%x, nsteps = %d\n",
        __FUNCTION__, als_zone,
        adp8862_data.bvalue[als_zone], bvalue[als_zone], nsteps);

    //adp8862_set_ramp (client, do_ramp, nsteps, &total_time);

    /* Write to each zone brightness register so that when it jumps into
     * the next zone the value is adjusted automatically
     */
    for (i = 0; i < ADP8862_NUM_ZONES; i++) {
        ret = adp8862_write_reg (adp8862_data.zone_reg[i], bvalue[i],
            __FUNCTION__);
        /* Remember the new value */
        adp8862_data.bvalue[i] = bvalue[i];
    }

    if (value == 0) {
        /* Disable everything */
        if (do_ramp) {
            /* Wait for ramping to finish */
            udelay (total_time);
        }
        adp8862_enable (client, 0);
    }

    mutex_unlock (&adp8862_mutex);
}
EXPORT_SYMBOL(adp8862_brightness_set);

static int adp8862_als_open (struct inode *inode, struct file *file)
{
    if (!adp8862_data.initialized)
        return -ENODEV;

    return 0;
}

static int adp8862_als_release (struct inode *inode, struct file *file)
{
    return 0;
}

#define CMD_LEN 5
static ssize_t adp8862_als_write (struct file *fp, const char __user *buf,
    size_t count, loff_t *pos)
{
    unsigned char cmd[CMD_LEN];
    int len;

    if (!adp8862_data.initialized)
        return -ENODEV;

    if (count < 1)
        return 0;

    len = count > CMD_LEN-1 ? CMD_LEN-1 : count;

    if (copy_from_user (cmd, buf, len))
        return -EFAULT;

    if (adp8862_data.revision <= 1)
        return -EFAULT;

    cmd[len] = '\0';
    if (cmd[len-1] == '\n') {
        cmd[len-1] = '\0';
        len--;
    }
    if (!strcmp (cmd, "1")) {
        printk (KERN_ERR "%s: enabling ALS\n", __FUNCTION__);
        mutex_lock (&adp8862_mutex);
        atomic_set (&adp8862_data.use_als, 1);
        /* No need to change ALS zone; interrupt handler will do it */
        //adp8862_write_reg (ADP8862_CONFIG_REG, value, __FUNCTION__);
        mutex_unlock (&adp8862_mutex);
    } else if (!strcmp (cmd, "0")) {
        printk (KERN_ERR "%s: disabling ALS\n", __FUNCTION__);
        mutex_lock (&adp8862_mutex);
        //adp8862_write_reg (ADP8862_CONFIG_REG, value, __FUNCTION__);
        atomic_set (&adp8862_data.use_als, 0);
        atomic_set (&adp8862_data.als_zone, ALS_NO_ZONE);
        adp8862_send_als_event (ALS_NO_ZONE);
        mutex_unlock (&adp8862_mutex);
    } else {
        printk (KERN_ERR "%s: invalid command %s\n", __FUNCTION__, cmd);
        return -EFAULT;
    }

    return count;
}

static ssize_t adp8862_als_read (struct file *file, char __user *buf,
    size_t count, loff_t *ppos)
{
    char z[10];
    unsigned zone;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    if (file->private_data)
        return 0;

    if (!atomic_read (&adp8862_data.use_als)) {
        sprintf (z, "%d\n", ALS_NO_ZONE);
    } else {
        zone = atomic_read (&adp8862_data.als_zone);
        sprintf (z, "%d\n", zone);
    }
    if (copy_to_user (buf, z, strlen (z)))
        return -EFAULT;

    file->private_data = (void *)1;
    return strlen (z);
}

static const struct file_operations als_fops = {
    .owner      = THIS_MODULE,
    .read       = adp8862_als_read,
    .write      = adp8862_als_write,
    .open       = adp8862_als_open,
    .release    = adp8862_als_release,
};

static struct miscdevice als_miscdev = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "als",
    .fops       = &als_fops,
};

/* This function is called by i2c_probe */
static int adp8862_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int ret = 0;
#ifdef CONFIG_MACH_MOT
	if (machine_is_morrison()) {
		printk(KERN_ERR "%s: not Morrison; returning\n", __FUNCTION__);
		return -EINVAL;
	}
#endif
#if 0
    if (!machine_is_zeppelin ()) {
        printk (KERN_ERR "%s: not zeppelin; returning\n",
            __FUNCTION__);
        return -EINVAL;
    }
#endif
    printk (KERN_ERR "%s: enter, I2C address = 0x%x, flags = 0x%x\n",
        __FUNCTION__, client->addr, client->flags);

    /* We should be able to read and write byte data */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
            __FUNCTION__);
        return -ENOTSUPP;
    }
    pr_debug ("%s: I2C_FUNC_I2C is supported\n", __FUNCTION__);

    adp8862_data.client = client;
    i2c_set_clientdata (client, &adp8862_data);

    /* Initialize chip */
    adp8862_setup (adp8862_data.client);

    /* Initialize interrupts */
    INIT_WORK(&adp8862_data.work, adp8862_work_func);
    if (client->irq) {
        ret = request_irq (client->irq, adp8862_irq_handler, IRQF_TRIGGER_LOW,
            "adp8862", &adp8862_data);

        if (ret == 0) {
            adp8862_data.use_irq = 1;
            ret = set_irq_wake (client->irq, 1);
        } else {
            printk (KERN_ERR "request_irq %d for adp8862 failed: %d\n",
                client->irq, ret);
            free_irq (client->irq, &adp8862_data);
            adp8862_data.use_irq = 0;
        }
    }

    /* Register LED class */
    ret = led_classdev_register (&client->adapter->dev, &adp8862_led);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, adp8862_led.name, ret);
        return ret;
    }

    /* Register LED class for no ramping */
    ret = led_classdev_register (&client->adapter->dev, &adp8862_led_noramp);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, adp8862_led.name, ret);
    }

    if ((ret = misc_register (&als_miscdev))) {
        printk (KERN_ERR "%s: misc_register failed, error %d\n",
            __FUNCTION__, ret);
        led_classdev_unregister (&adp8862_led);
        led_classdev_unregister (&adp8862_led_noramp);
        return ret;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend (&early_suspend_data);
    pr_debug ("%s: registered with android PM\n", __FUNCTION__);
#endif

    adp8862_data.initialized = 1;

    return 0;
}

static irqreturn_t adp8862_irq_handler (int irq, void *dev_id)
{
    struct adp8862 *data_ptr = (struct adp8862 *)dev_id;

    pr_debug ("%s: got an interrupt %d\n", __FUNCTION__, irq);

    disable_irq (irq);
    schedule_work (&data_ptr->work);

    return IRQ_HANDLED;
}

static void adp8862_send_als_event (int zone)
{
    char event_string[20];
    char *envp[] = {event_string, NULL};
    int ret;

    sprintf (event_string, "ALS_ZONE=%d", zone);
    ret = kobject_uevent_env (&als_miscdev.this_device->kobj,
        KOBJ_CHANGE, envp);
    if (ret)
        printk (KERN_ERR "%s: kobject_uevent_env failed: %d\n",
            __FUNCTION__, ret);
}
#if 0
static void adp8862_call_als_callbacks (unsigned old_zone, unsigned zone)
{
    struct als_callback *c;

    mutex_lock (&als_cb_mutex);
    list_for_each_entry(c, &als_callbacks, entry) {
        c->cb (old_zone, zone, c->cookie);
    }
    mutex_unlock (&als_cb_mutex);
}
#endif
static void adp8862_work_func (struct work_struct *work)
{
#if 0
    int ret;
    uint8_t reg;
    unsigned zone, old_zone;
#endif

    pr_debug ("%s: work function called\n", __FUNCTION__);
#if 0
    ret = adp8862_read_reg (ADP8862_ALS_REG, &reg);
    if (ret) {
        pr_debug ("%s: ALS Register: 0x%x\n", __FUNCTION__, reg);
        if (reg & ALS_FLAG_MASK) {
            zone = reg & ALS_ZONE_MASK;
            old_zone = atomic_read (&adp8862_data.als_zone);
            printk (KERN_INFO "%s: ALS zone changed: %d => %d\n",
                __FUNCTION__, old_zone, zone);
            atomic_set (&adp8862_data.als_zone, zone);
            adp8862_send_als_event (zone);
            adp8862_call_als_callbacks (old_zone, zone);
        }
    }
#endif
    enable_irq (adp8862_data.client->irq);
}

#if 0
static enum hrtimer_restart adp8862_timer_func (struct hrtimer *timer)
{
    schedule_work(&adp8862_data.work);

    hrtimer_start(&adp8862_data.timer,
        ktime_set(1, 0), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}
#endif

#if 0
/* This function calculates ramp step time so that total ramp time is
 * 0.5 sec = 500ms
 */
#define TOTAL_RAMP 500000 // Total ramp time in microseconds
static int adp8862_set_ramp (struct i2c_client *client,
    unsigned int on, unsigned int nsteps, unsigned int *rtime)
{
    int ret, i = 0;
    uint8_t value = 0;
    unsigned int total_time = 0;

    if (on) {
        /* Calculate the closest possible ramp time */
        for (i = 0; i < adp8862_data.nramp; i++) {
            total_time = nsteps * adp8862_ramp[i];
            if (total_time >= TOTAL_RAMP)
                break;
        }
        if (i > 0) {
            /* If previous value is closer */
            if (total_time - TOTAL_RAMP >
                    TOTAL_RAMP - nsteps * adp8862_ramp[i-1]) {
                i--;
                total_time = nsteps * adp8862_ramp[i];
            }
        }
        adp8862_set_options_f (&value, i);
    }

#if 0
    printk (KERN_ERR "%s: ramp = %s, ramp step = %d us (total = %d us)\n",
        __FUNCTION__, on ? "on" : "off", adp8862_ramp[i], total_time);
#endif
    if (rtime)
        *rtime = total_time;
    ret = adp8862_write_reg (ADP8862_OPTIONS_REG, value, __FUNCTION__);

    return ret;
}
#endif

static int adp8862_enable (struct i2c_client *client, unsigned int on)
{
    int ret;
    uint8_t value = 0x21;

    if (!on)
        value = 0x20;

    ret = adp8862_write_reg (ADP8862_MDCR_REG, value, __FUNCTION__);
    adp8862_data.enabled = on;
    return ret;
}

static int adp8862_configure (void)
{
    int ret = 0;

    /* Disable all interrupts.  Put 0x1 for ALS */
    adp8862_write_reg (ADP8862_INTREN_REG, 0x0, __FUNCTION__);
    /* Connect all LED sinks to BL_EN in MDCR register */
    adp8862_write_reg (ADP8862_BLSEN_REG, 0x0, __FUNCTION__);
    /* Daylight mode, linear law DAC, fade override disabled */
    adp8862_write_reg (ADP8862_CFGR_REG, 0x0, __FUNCTION__);
    /* Disable backlight off timeout, it's done by the framework */
    adp8862_write_reg (ADP8862_BLOFF_REG, 0x0, __FUNCTION__);
    /* Disable backlight dim timeout, it's done by the framework */
    adp8862_write_reg (ADP8862_BLDIM_REG, 0x0, __FUNCTION__);

    return ret;
}

static int adp8862_setup (struct i2c_client *client)
{
    int ret;
    uint8_t value = 0;

    /* Read revision number */
    ret = adp8862_read_reg (ADP8862_MFDVID_REG, &value);
    if (ret < 0) {
        printk (KERN_ERR "%s: unable to read from chip: %d\n",
            __FUNCTION__, ret);
        return ret;
    }
    adp8862_data.revision = value;
    /* revision is going to be an index to adp8862_ramp array */
    printk (KERN_ERR "%s: MFDVID 0x%X\n",
        __FUNCTION__, adp8862_data.revision);

    /* No ALS for now */
    atomic_set (&adp8862_data.use_als, 0);
    ret = adp8862_configure ();
    if (ret < 0)
        return ret;

    ret = adp8862_enable (client, 1);
    if (ret < 0)
        return ret;

    //hrtimer_init (&adp8862_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //adp8862_data.timer.function = adp8862_timer_func;
    //hrtimer_start(&adp8862_data.timer, ktime_set(2, 0), HRTIMER_MODE_REL);

    return ret;
}

static int adp8862_remove (struct i2c_client *client)
{
    struct adp8862 *data_ptr = i2c_get_clientdata(client);
    if (data_ptr->use_irq)
        free_irq (client->irq, data_ptr);
    led_classdev_unregister (&adp8862_led);
    led_classdev_unregister (&adp8862_led_noramp);
    return 0;
}

static int adp8862_suspend (struct i2c_client *client, pm_message_t mesg)
{
    pr_info ("%s: called with pm message %d\n", __FUNCTION__, mesg.event);
    led_classdev_suspend (&adp8862_led);
    return 0;
}

static int adp8862_resume (struct i2c_client *client)
{
    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    mutex_lock (&adp8862_mutex);
    adp8862_configure ();
    mutex_unlock (&adp8862_mutex);
    led_classdev_resume (&adp8862_led);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp8862_early_suspend (struct early_suspend *h)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    adp8862_suspend (adp8862_data.client, PMSG_SUSPEND);
}

static void adp8862_late_resume (struct early_suspend *h)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    adp8862_resume (adp8862_data.client);
}
#endif


static int __devinit adp8862_init (void)
{
    int ret;

    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    ret = i2c_add_driver (&adp8862_driver);
    if (ret) {
        printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n",
            __FUNCTION__, ret);
    }

    return ret;
}

static void __exit adp8862_exit(void)
{
    i2c_del_driver (&adp8862_driver);
}

static char *reg_name (int reg)
{
    switch (reg) {
        case (ADP8862_MFDVID_REG): return "MFDVID"; break;
        case (ADP8862_MDCR_REG): return "MDCR"; break;
        case (ADP8862_MDCR2_REG): return "MDCR2"; break;
        case (ADP8862_INTREN_REG): return "INTREN"; break;
        case (ADP8862_CFGR_REG): return "CFGR"; break;
        case (ADP8862_BLSEN_REG): return "BLSEN"; break;
        case (ADP8862_BLOFF_REG): return "BLOFF"; break;
        case (ADP8862_BLDIM_REG): return "BLDIM"; break;
        case (ADP8862_BLFR_REG): return "BLFR"; break;
        case (ADP8862_BLMX1_REG): return "BLMX1"; break;
        case (ADP8862_BLDM1_REG): return "BLDM1"; break;
        case (ADP8862_BLMX2_REG): return "BLMX2"; break;
        case (ADP8862_BLDM2_REG): return "BLDM2"; break;
        case (ADP8862_BLMX3_REG): return "BLMX3"; break;
        case (ADP8862_BLDM3_REG): return "BLDM3"; break;
        default: return "UNKNOWN"; break;
    }
    return "UNKNOWN";
}

unsigned adpxxxx_detect_esd (void)
{
    //uint8_t value = 0;

    //0 - no ESD, 1 - ESD
    return 0;
}
EXPORT_SYMBOL(adpxxxx_detect_esd);

void adpxxxx_fix_esd (void)
{
    mutex_lock (&adp8862_mutex);
    adp8862_configure ();
    mutex_unlock (&adp8862_mutex);
    return;
}
EXPORT_SYMBOL(adpxxxx_fix_esd);

module_init(adp8862_init);
module_exit(adp8862_exit);

MODULE_DESCRIPTION("ADP8862 DISPLAY BACKLIGHT DRIVER");
MODULE_LICENSE("GPL v2");
