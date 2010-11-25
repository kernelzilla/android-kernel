/*
 * Copyright (C) 2009 Motorola, Inc.
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

/*=============================================================================
                       CONSTANTS
=============================================================================*/
#define ADP8860_STR_NAME      "adp8860"
#define ADP8860_MFDVID_REG    0x00
#define ADP8860_MDCR_REG      0x01
#define ADP8860_MDCR2_REG     0x02
#define ADP8860_INTR_EN_REG   0x03
#define ADP8860_CFGR_REG      0x04
#define ADP8860_BLSEN_REG     0x05
#define ADP8860_BLOFF_REG     0x06
#define ADP8860_BLDIM_REG     0x07
#define ADP8860_BLFR_REG      0x08
#define ADP8860_BLMX1_REG     0x09
#define ADP8860_BLDM1_REG     0x0A
#define ADP8860_BLMX2_REG     0x0B
#define ADP8860_BLDM2_REG     0x0C
#define ADP8860_BLMX3_REG     0x0D
#define ADP8860_BLDM3_REG     0x0E
#define ADP8860_ISCFR_REG     0x0F
#define ADP8860_ISCC_REG      0x10
#define ADP8860_ISCT1_REG     0x11
#define ADP8860_ISCT2_REG     0x12
#define ADP8860_ISCF_REG      0x13
#define ADP8860_ISC7_REG      0x14
#define ADP8860_ISC6_REG      0x15
#define ADP8860_ISC5_REG      0x16
#define ADP8860_ISC4_REG      0x17
#define ADP8860_ISC3_REG      0x18
#define ADP8860_ISC2_REG      0x19
#define ADP8860_ISC1_REG      0x1A
#define ADP8860_CCFG_REG      0x1B
#define ADP8860_CCFG2_REG     0x1C
#define ADP8860_L2_TRP_REG    0x1D
#define ADP8860_L2_HYS_REG    0x1E
#define ADP8860_L3_TRP_REG    0x1F
#define ADP8860_L3_HYS_REG    0x20
#define ADP8860_PH1LEVL_REG   0x21
#define ADP8860_PH1LEVH_REG   0x22
#define ADP8860_PH2LEVL_REG   0x23
#define ADP8860_PH2LEVH_REG   0x24

#define ADP8860_REG_WRITE_LEN 2
#define ADP8860_STATE_INIT    0
#define ADP8860_STATE_ACTIVE  1
#define ADP8860_STATE_STANDBY 2

#define ADP8860_BLSEN_1TO6_EN 0x40
#define ADP8860_MDCR_MANUAL   0x21
#define ADP8860_BLOFF_NONE    0x00
#define ADP8860_BLDIM_NONE    0x00


#ifndef FALSE
#define FALSE  0
#define TRUE   (!FALSE)
#endif

/*=============================================================================
                       MACROS
=============================================================================*/
#define ADP8860_PRINTK(fmt, ...) \
   if (adp8860_debug) \
      printk(fmt, ##__VA_ARGS__)

/*=============================================================================
                   LOCAL FUNCTION PROTOTYPES
=============================================================================*/
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp8860_early_suspend (struct early_suspend *h);
static void adp8860_late_resume (struct early_suspend *h);
static struct early_suspend early_suspend_data = {
   .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5,
   .suspend = adp8860_early_suspend,
   .resume = adp8860_late_resume,
};
#endif
static int adp8860_suspend (struct i2c_client *client, pm_message_t mesg);
static int adp8860_resume (struct i2c_client *client);
#endif
static void adp8860_brightness_set (struct led_classdev *led_cdev,
   enum led_brightness value);
static void adp8860_brightness_set_nr (struct led_classdev *led_cdev,
   enum led_brightness value);
static int adp8860_probe (struct i2c_client *client,
   const struct i2c_device_id *id);
static int adp8860_write_reg (unsigned reg, uint8_t value, const char *caller);
static int adp8860_remove (struct i2c_client *client);

/*=============================================================================
                      LOCAL STRUCTURES, UNIONS, ENUMS PROTOTYPES
=============================================================================*/
struct adp8860
{
   struct i2c_client *client;
};

/*=============================================================================
                       LOCAL VARIABLES
=============================================================================*/
static bool adp8860_debug = TRUE;
static uint8_t adp8860_state = ADP8860_STATE_INIT;

static struct adp8860 adp8860_data = {
};

/* LED class struct */
static struct led_classdev adp8860_led = {
   .name = "lcd-backlight",
   .brightness_set = adp8860_brightness_set,
};

/* LED class struct for no ramping */
static struct led_classdev adp8860_led_noramp = {
   .name = "lcd-nr-backlight",
   .brightness_set = adp8860_brightness_set_nr,
};

static const struct i2c_device_id adp8860_id[] = {
   { ADP8860_STR_NAME, 0 },
   { }
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver adp8860_driver =
{
   .driver = {
      .name   = ADP8860_STR_NAME,
   },
   .id_table = adp8860_id,
   .probe = adp8860_probe,
   .remove  = adp8860_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
   .suspend   = adp8860_suspend,
   .resume    = adp8860_resume,
#endif
};

static int adp8860_read_reg (unsigned reg, uint8_t *value)
{
   struct i2c_client *client = adp8860_data.client;
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

static int adp8860_write_reg (unsigned reg, uint8_t value, const char *caller)
{
   uint8_t buf[ADP8860_REG_WRITE_LEN] = { reg, value };
   int ret = 0;

   ADP8860_PRINTK ("%s: writing 0x%X to reg 0x%X at addr 0x%X\n",
      caller, buf[1], buf[0], adp8860_data.client->addr);
   ret = i2c_master_send (adp8860_data.client, buf, ADP8860_REG_WRITE_LEN);
   if (ret != ADP8860_REG_WRITE_LEN)
   {
      printk (KERN_ERR "%s: i2c_master_send error %d\n",
         caller, ret);
      if (ret >= 0)
      {
         /* Entire message wasn't written. Try again. */
         ret = -EAGAIN;
      }
   }
   else
   {
      ret = 0;
   }
   return ret;
}

static int adp8860_device_init(void)
{
   int ret = 0;

   ADP8860_PRINTK ("%s: Enter\n", __FUNCTION__);

   /* disable the backlight dim timeout */
   ret = adp8860_write_reg(ADP8860_BLDIM_REG, ADP8860_BLDIM_NONE, __FUNCTION__);
   /* disable the backlight OFF timeout */
   if (!ret)
   {
      ret = adp8860_write_reg(ADP8860_BLOFF_REG, ADP8860_BLOFF_NONE,
         __FUNCTION__);
   }
   /* connect LEDs 1-6 to BL_EN */
   if (!ret)
   {
      adp8860_write_reg(ADP8860_BLSEN_REG, ADP8860_BLSEN_1TO6_EN, __FUNCTION__);
   }
   /* enable backlight in manual mode */
   if (!ret)
   {
      adp8860_write_reg(ADP8860_MDCR_REG, ADP8860_MDCR_MANUAL, __FUNCTION__);
   }

   if (ret)
   {
      printk (KERN_ERR "%s: failed\n", __FUNCTION__);
   }
   else
   {
      adp8860_state = ADP8860_STATE_ACTIVE;
   }
   ADP8860_PRINTK ("%s: Exit, adp8860_state=%d, ret=%d\n",
      __FUNCTION__, adp8860_state, ret);

   return ret;
}

static void adp8860_set_brightness (struct led_classdev *led_cdev,
   enum led_brightness value, bool ramp)
{
   ADP8860_PRINTK ("%s: led_cdev->name=%s value=%d ramp=%d\n",
      __FUNCTION__, led_cdev->name, value, ramp);

   if (adp8860_state == ADP8860_STATE_INIT)
   {
      adp8860_device_init();
   }

   if (value > LED_FULL)
   {
      value = LED_FULL;
   }

   /* set brightness level */
   adp8860_write_reg(ADP8860_BLMX1_REG, (value >> 1), __FUNCTION__);
}

static void adp8860_brightness_set (struct led_classdev *led_cdev,
   enum led_brightness value)
{
   adp8860_set_brightness(led_cdev, value, TRUE);
}

static void adp8860_brightness_set_nr (struct led_classdev *led_cdev,
   enum led_brightness value)
{
   adp8860_set_brightness(led_cdev, value, FALSE);
}

/* This function is called by i2c_probe */
static int adp8860_probe (struct i2c_client *client,
   const struct i2c_device_id *id)
{
   int ret = 0;

   printk ("%s: enter, I2C address = 0x%x, flags = 0x%x\n", 
      __FUNCTION__, client->addr, client->flags);

        if (i2c_smbus_xfer(client->adapter, client->addr, 0, 0, 0,
                       I2C_SMBUS_QUICK, NULL) < 0) {
printk (KERN_ERR "%s: device at address %x, not present\n", __FUNCTION__, client->addr);
                return -ENODEV;
        }
   /* We should be able to read and write byte data */
   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
      printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
         __FUNCTION__);
      return -ENOTSUPP;
   }

   adp8860_data.client = client;
   i2c_set_clientdata (client, &adp8860_data);

   /* Register LED class */
   ret = led_classdev_register (&client->adapter->dev, &adp8860_led);
   if (ret) {
      printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
         __FUNCTION__, adp8860_led.name, ret);
      return ret;
   } 

   /* Register LED class for no ramping */
   ret = led_classdev_register (&client->adapter->dev, &adp8860_led_noramp);
   if (ret) {
      printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
         __FUNCTION__, adp8860_led.name, ret);
      return ret;
   } 

#ifdef CONFIG_HAS_EARLYSUSPEND
   register_early_suspend (&early_suspend_data);
#endif

   /* TEMP HACK - Set brightness to 3/4 LED_FULL */
#ifndef TEMP_7XXX_CODE_COMPLETE
   adp8860_brightness_set_nr(&adp8860_led_noramp, (int)(LED_FULL * (3.0/4.0)));
#endif /* TEMP_7XXX_CODE_COMPLETE */

   printk ("%s: exit, ret=%d\n", __FUNCTION__, ret);

   return 0;
}

static int adp8860_remove (struct i2c_client *client)
{
   struct adp8860 *data_ptr = i2c_get_clientdata(client);
   led_classdev_unregister(&adp8860_led);
   led_classdev_unregister(&adp8860_led_noramp);
   return 0;
}

static int adp8860_suspend (struct i2c_client *client, pm_message_t mesg)
{
   ADP8860_PRINTK("%s: called with pm message %d\n", 
     __FUNCTION__, mesg.event);
   led_classdev_suspend (&adp8860_led);
   return 0;
}

static int adp8860_resume (struct i2c_client *client)
{
   printk("%s: resuming\n", __FUNCTION__);
   led_classdev_resume (&adp8860_led);
   printk("%s: driver resumed\n", __FUNCTION__);
   return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp8860_early_suspend (struct early_suspend *h)
{
   adp8860_suspend (adp8860_data.client, PMSG_SUSPEND);
}

static void adp8860_late_resume (struct early_suspend *h)
{
   adp8860_resume (adp8860_data.client);
}
#endif

static void adp8860_set_debug(bool on)
{
   adp8860_debug = on;
}

static int __devinit adp8860_driver_init (void)
{
   int ret;

   printk (KERN_INFO "%s: enter\n", __FUNCTION__);
   ret = i2c_add_driver (&adp8860_driver);
   if (ret) {
      printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n", 
         __FUNCTION__, ret);
   }

   return ret;
}

static void __exit adp8860_driver_exit(void)
{
   i2c_del_driver (&adp8860_driver);
}

module_init(adp8860_driver_init);
module_exit(adp8860_driver_exit);

MODULE_DESCRIPTION("ADP8860 DISPLAY BACKLIGHT DRIVER");
MODULE_AUTHOR("Ryan C Johnson <ryan.johnson@motorola.com>")
MODULE_LICENSE("GPL v2");

