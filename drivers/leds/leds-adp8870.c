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
 * Motorola 2010-Jan-26 add support for messaging led
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
#include "board-mot.h"
#include <linux/wakelock.h>

#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*=============================================================================
                       CONSTANTS
=============================================================================*/
#define ADP8870_STR_NAME      "adp8870"
#define ADP8870_MFDVID_REG    0x00
#define ADP8870_MDCR_REG      0x01
#define ADP8870_INTR_EN_REG   0x03
#define ADP8870_CFGR_REG      0x04
#define ADP8870_BLSEN_REG     0x05
#define ADP8870_PWMLED_REG    0x06
#define ADP8870_BLOFF_REG     0x07
#define ADP8870_BLDIM_REG     0x08
#define ADP8870_BLFR_REG      0x09
#define ADP8870_BLMX1_REG     0x0A
#define ADP8870_BLDM1_REG     0x0B
#define ADP8870_BLMX2_REG     0x0C
#define ADP8870_BLDM2_REG     0x0D
#define ADP8870_BLMX3_REG     0x0E
#define ADP8870_BLDM3_REG     0x0F
#define ADP8870_ISCC_REG      0x1B
#define ADP8870_ISCT1_REG     0x1C
#define ADP8870_ISCT2_REG     0x1D
#define ADP8870_ISCF_REG      0x1E
#define ADP8870_ISC7_REG      0x25
#define ADP8870_ISC6_REG      0x24
#define ADP8870_ISC5_REG      0x23
#define ADP8870_ISC4_REG      0x22
#define ADP8870_ISC3_REG      0x21
#define ADP8870_ISC2_REG      0x20
#define ADP8870_ISC1_REG      0x1F
#define ADP8870_L2_TRP_REG    0x32
#define ADP8870_L2_HYS_REG    0x33
#define ADP8870_L3_HYS_REG    0x35
#define ADP8870_INTSTAT_REG   0x02
#define ADP8870_BLMX4_REG     0x10
#define ADP8870_BLDM4_REG     0x11
#define ADP8870_BLMX5_REG     0x12
#define ADP8870_BLDM5_REG     0x13
#define ADP8870_ISCLAW_REG    0x1A
#define ADP8870_ISC7L2_REG    0x26
#define ADP8870_ISC7L3_REG    0x27
#define ADP8870_ISC7L4_REG    0x28
#define ADP8870_ISC7L5_REG    0x29
#define ADP8870_CMPCTL_REG    0x2D
#define ADP8870_ALS1EN_REG    0x2E
#define ADP8870_ALS2EN_REG    0x2F
#define ADP8870_ALS1STAT_REG  0x30
#define ADP8870_ALS2STAT_REG  0x31
#define ADP8870_L4TRP_REG     0x36
#define ADP8870_L4HYS_REG     0x37
#define ADP8870_L5TRP_REG     0x38
#define ADP8870_L5HYS_REG     0x39

#define ADP8870_REG_WRITE_LEN 2



#define ADP8870_BLSEN_1TO6_EN 0x40
#define ADP8870_PWMLED_1TO6_ON 0x3F
#define ADP8870_PWMLED_1TO6_OFF 0x00
#define ADP8870_MDCR_MANUAL   0x21
#define ADP8870_BLOFF_NONE    0x00
#define ADP8870_BLDIM_NONE    0x00

typedef struct blink_data {
    uint32_t   time_on;
    uint32_t   time_off;
    uint8_t    brightness;
}blink_data_t;





#ifndef FALSE
#define FALSE  0
#define TRUE   (!FALSE)
#endif

#define ALS_MODE_ON 1
#define ALS_MODE_OFF 0
#define PWM_MODE_ON 1
#define PWM_MODE_OFF 0

/*=============================================================================
                       MACROS
=============================================================================*/
#define ADP8870_PRINTK(fmt, ...) \
   if (adp8870_debug) \
      printk(fmt, ##__VA_ARGS__)

/*=============================================================================
                   LOCAL FUNCTION PROTOTYPES
=============================================================================*/
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp8870_early_suspend (struct early_suspend *h);
static void adp8870_late_resume (struct early_suspend *h);
static struct early_suspend early_suspend_data = {
   .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5,
   .suspend = adp8870_early_suspend,
   .resume = adp8870_late_resume,
};
#endif
static int adp8870_suspend (struct i2c_client *client, pm_message_t mesg);
static int adp8870_resume (struct i2c_client *client);
#endif
static void adp8870_brightness_set (struct led_classdev *led_cdev,
   enum led_brightness value);
static void adp8870_brightness_set_nr (struct led_classdev *led_cdev,
   enum led_brightness value);
static int adp8870_probe (struct i2c_client *client,
   const struct i2c_device_id *id);
static int adp8870_write_reg (unsigned reg, uint8_t value, const char *caller);
static int adp8870_remove (struct i2c_client *client);
static int adp8870_sub_led_blink_set (/*struct led_classdev *led_cdev,*/
				     int delay_on,
				     int delay_off);
static void adp8870_sub_led_brightness_set (struct led_classdev *led_cdev,
   enum led_brightness value);
static void adp8870_set_sub_led_brightness(uint8_t brightness);
static int adp8870_device_init(void);

/*=============================================================================
                      LOCAL STRUCTURES, UNIONS, ENUMS PROTOTYPES
=============================================================================*/




typedef enum 
{
  BLINK_OFF,
  BLINK_ON,
  MAIN_LED_OFF,
  MAIN_LED_ON,
  BLINK_STATE_END
}blink_state_t;


typedef enum
{
  ADP8870_STATE_INIT,
  ADP8870_STATE_ALL_LEDS_ACTIVE,
  ADP8870_STATE_MAIN_LEDS_ACTIVE,
  ADP8870_STATE_SUB_LEDS_ACTIVE,
  ADP8870_STATE_END
}adp8870_state_t;

#define ADP8870_DARK_MIN_SETTING 0x18
#define ADP8870_INDOOR_WEAK_MIN_SETTING 0x29
#define ADP8870_INDOOR_NORMAL_MIN_SETTING 0x35
#define ADP8870_INDOOR_STRONG_MIN_SETTING 0x3F
#define ADP8870_OUTDOOR_NORMAL_MIN_SETTING 0x4E

#define ADP8870_DARK_TYP_SETTING 0x1C
#define ADP8870_INDOOR_WEAK_TYP_SETTING 0x2D
#define ADP8870_INDOOR_NORMAL_TYP_SETTING 0x39
#define ADP8870_INDOOR_STRONG_TYP_SETTING 0x43
#define ADP8870_OUTDOOR_NORMAL_TYP_SETTING 0x52

#define ADP8870_DARK_MAX_SETTING 0x20
#define ADP8870_INDOOR_WEAK_MAX_SETTING 0x31
#define ADP8870_INDOOR_NORMAL_MAX_SETTING 0x3C
#define ADP8870_INDOOR_STRONG_MAX_SETTING 0x47
#define ADP8870_OUTDOOR_NORMAL_MAX_SETTING 0x56

#define STATE_ON 1
#define STATE_OFF 0
#define UPDATE_BLINK_STATE(state) do {state = 1-state;} while(0)

/*below is copy from settings applicaiton*/
#define  BRIGHTNESS_DIM  20 
#define  BRIGHTNESS_ON  255
#define  BRIGHTNESS_MIN_SETTING 30
#define  BRIGHTNESS_MAX_SETTING 255
#define  BRIGHTNESS_TYP_SETTING ((BRIGHTNESS_MAX_SETTING  - BRIGHTNESS_MIN_SETTING)/2)


typedef struct range
{
   uint8_t min;
   uint8_t typ;
   uint8_t max;
}range_t;


typedef struct adp8870
{
   struct i2c_client *client;   
   struct work_struct sub_led_blink_wq;   
   ktime_t    sub_led_blink_on_time;
   ktime_t    sub_led_blink_off_time;
   struct hrtimer sub_led_blink_timer;
   struct wake_lock wake_lock;
   uint8_t  sub_led_brighness;
   uint8_t initialized;
   uint8_t state;
   uint8_t blink_state;
   uint8_t user_setting_brightness;
   uint8_t als_mode;    
   uint8_t pwm_mode;
}adp8870_data_t;

/*=============================================================================
                       LOCAL VARIABLES
=============================================================================*/
static uint adp8870_debug = TRUE;
module_param(adp8870_debug, uint, 0644);
static DEFINE_MUTEX(adp8870_mutex);
static struct adp8870 adp8870_data = {
	.initialized = 0,
};




static uint8_t adp8870_state = ADP8870_STATE_INIT;


adp8870_state_t adp8870_blink_sm[ADP8870_STATE_END][BLINK_STATE_END] =
{
    {ADP8870_STATE_INIT, ADP8870_STATE_SUB_LEDS_ACTIVE,ADP8870_STATE_INIT, ADP8870_STATE_MAIN_LEDS_ACTIVE}, /*init state: ADP8870_STATE_SUB_LEDS_ACTIVE*/
    {ADP8870_STATE_MAIN_LEDS_ACTIVE, ADP8870_STATE_ALL_LEDS_ACTIVE,ADP8870_STATE_SUB_LEDS_ACTIVE, ADP8870_STATE_ALL_LEDS_ACTIVE}, /*ADP8870_STATE_ALL_LEDS_ACTIVE*/
    {ADP8870_STATE_MAIN_LEDS_ACTIVE, ADP8870_STATE_ALL_LEDS_ACTIVE,ADP8870_STATE_INIT,ADP8870_STATE_MAIN_LEDS_ACTIVE }, /*ADP8870_STATE_MAIN_LEDS_ACTIVE*/
    {ADP8870_STATE_INIT, ADP8870_STATE_SUB_LEDS_ACTIVE,ADP8870_STATE_SUB_LEDS_ACTIVE, ADP8870_STATE_ALL_LEDS_ACTIVE }, /*ADP8870_STATE_SUB_LEDS_ACTIVE*/
};

/* LED class struct */
static struct led_classdev adp8870_led = {
   .name = "lcd-backlight",
   .brightness_set = adp8870_brightness_set,
};

static struct led_classdev adp8870_msg_led = {
    .name = "messaging",
    .brightness_set = adp8870_sub_led_brightness_set,
};

uint8_t conv_setting_to_reg(uint8_t user_settings)
{
   uint8_t step = ADP8870_OUTDOOR_NORMAL_MAX_SETTING - ADP8870_DARK_MIN_SETTING;
   uint16_t covert_value = 0;
   
   if (user_settings == 0 )
	   return user_settings;
	   
   covert_value = ADP8870_OUTDOOR_NORMAL_MAX_SETTING - (BRIGHTNESS_MAX_SETTING- user_settings)*step/(BRIGHTNESS_MAX_SETTING  - BRIGHTNESS_MIN_SETTING);
   return (uint8_t)covert_value;   
}


static ssize_t adp8870_blink_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    int on_time, off_time;

    on_time = adp8870_data.sub_led_blink_on_time.tv.sec+
            adp8870_data.sub_led_blink_on_time.tv.nsec/NSEC_PER_USEC;
    off_time = adp8870_data.sub_led_blink_off_time.tv.sec+
            adp8870_data.sub_led_blink_off_time.tv.nsec/NSEC_PER_USEC;

    
    if (!buf){
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    } 
    sprintf (buf, "on:%d  off:%d\n", on_time, off_time);
    return strlen(buf);
}

static ssize_t adp8870_blink_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    int n  = 0;
    int delay_on, delay_off;

   

    if (!buf || size == 0) 
        goto error_exit;    
    n = sscanf(buf, "%d %d", &delay_on, &delay_off);
    if ( n < 2)
        goto error_exit;
    ADP8870_PRINTK(KERN_INFO "adp8870 blink delay on %d, delay off %d\n",delay_on, delay_off); 
    adp8870_sub_led_blink_set(delay_on, delay_off);
    return size;    
    
error_exit:
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;    
}

static ssize_t adp8870_als_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    if (!buf){
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    sprintf(buf, "als:%d\n", adp8870_data.als_mode);
    return strlen(buf);
}

static ssize_t adp8870_als_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
	uint8_t als_mode = 0;
	int32_t n = 0;

	if (!buf || size == 0)
		goto error_exit;

	n = sscanf(buf, "%d", &als_mode);
	if (n < 1) {
		printk(KERN_ERR "n = %d, als_mode = %d\n", n, als_mode);
		goto error_exit;
	}

	adp8870_data.als_mode  = als_mode;
	return n;

error_exit:
	printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
	return -EINVAL;
}

static ssize_t adp8870_pwm_mode_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    if (!buf){
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    sprintf(buf, "pwm_mode:%d\n", adp8870_data.pwm_mode);
    return strlen(buf);
}

static ssize_t adp8870_pwm_mode_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
   uint8_t pwm_mode = 0;
   int32_t n = 0;

   if (!buf || size == 0) 
      goto error_exit;    

   n = sscanf(buf, "%d", &pwm_mode);
   if (n < 1)
      goto error_exit;

   adp8870_data.pwm_mode = pwm_mode;
   if (pwm_mode == PWM_MODE_ON) {      
      mutex_lock(&adp8870_mutex);
      adp8870_write_reg(ADP8870_PWMLED_REG, ADP8870_PWMLED_1TO6_ON , __FUNCTION__);
      mutex_unlock(&adp8870_mutex);
   } else if (pwm_mode == PWM_MODE_OFF) {
      mutex_lock(&adp8870_mutex);
      adp8870_write_reg(ADP8870_PWMLED_REG, ADP8870_PWMLED_1TO6_OFF, __FUNCTION__);     
      mutex_unlock(&adp8870_mutex);
   } else {
      goto error_exit;
   }      
   return n;

error_exit:
    printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
    return -EINVAL; 

}



static DEVICE_ATTR(blink, 0644, adp8870_blink_show, adp8870_blink_store);
static DEVICE_ATTR(als, 0644, adp8870_als_show, adp8870_als_store);
static DEVICE_ATTR(pwm_mode, 0644, adp8870_pwm_mode_show, adp8870_pwm_mode_store);

/* LED class struct for no ramping */
static struct led_classdev adp8870_led_noramp = {
   .name = "lcd-nr-backlight",
   .brightness_set = adp8870_brightness_set_nr,
};

static const struct i2c_device_id adp8870_id[] = {
   { ADP8870_STR_NAME, 0 },
   { }
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver adp8870_driver =
{
   .driver = {
      .name   = ADP8870_STR_NAME,
   },
   .id_table = adp8870_id,
   .probe = adp8870_probe,
   .remove  = adp8870_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
   .suspend   = adp8870_suspend,
   .resume    = adp8870_resume,
#endif
};


int adp8870_get_msg_led_status()
{
	return (adp8870_state == ADP8870_STATE_SUB_LEDS_ACTIVE ||
		adp8870_state == ADP8870_STATE_ALL_LEDS_ACTIVE)? 1 : 0;
}

/*
 *    bright : 0 on:xx off:xx  : turn off; bright: xx on:0, off:0, always light on, bright:xx on:xx, off:xx blink
 *    bright : xx on :0 off:xx and bright: xx on:xx off:0 turn off
*/


static void adp8870_state_update(blink_state_t new_event)
{
	adp8870_state_t new_state,old_state;

	mutex_lock(&adp8870_mutex);
	new_state = adp8870_blink_sm[adp8870_state][new_event];
	old_state = adp8870_state;
	adp8870_state = new_state;
	mutex_unlock(&adp8870_mutex);

	ADP8870_PRINTK(KERN_INFO"%s, prev state = %d, new state = %d\n",
			__FUNCTION__, old_state, new_state);

	if (new_state == old_state)
		return;
	if (old_state == ADP8870_STATE_INIT) {
		msleep_interruptible(5);
		ADP8870_PRINTK(KERN_INFO"%s, set gpio output 1\n",__FUNCTION__);
		gpio_set_value(LCD_RST_N_SIGNAL, 1);
		adp8870_device_init();
	}
 
	return;
}

static void adp8870_sub_led_brightness_set (struct led_classdev *led_cdev,
						enum led_brightness value)
{
	adp8870_data.sub_led_brighness = value;
	if (adp8870_data.sub_led_brighness == 0) {    /*turn off sub leds*/
		ADP8870_PRINTK(KERN_INFO"%s, brightness %d, cancel timer\n",
				__FUNCTION__,adp8870_data.sub_led_brighness);
		if (adp8870_state != ADP8870_STATE_INIT)
			adp8870_set_sub_led_brightness(0);
		adp8870_state_update(BLINK_OFF);
		wake_unlock(&adp8870_data.wake_lock);
		hrtimer_cancel(&adp8870_data.sub_led_blink_timer);
	} else {  /*brightness isn't 0*/
		if (ktime_to_ns(adp8870_data.sub_led_blink_on_time) != 0 &&
			ktime_to_ns(adp8870_data.sub_led_blink_off_time) != 0) {
			/*start blink*/
			adp8870_state_update(BLINK_ON);
			wake_lock(&adp8870_data.wake_lock);
			hrtimer_start(&adp8870_data.sub_led_blink_timer,
					adp8870_data.sub_led_blink_on_time,
					HRTIMER_MODE_REL);
			ADP8870_PRINTK(KERN_INFO"%s, brightness %d, start timer\n",
					__FUNCTION__,
					adp8870_data.sub_led_brighness);
		} else if (ktime_to_ns(adp8870_data.sub_led_blink_on_time) == 0 &&
			ktime_to_ns(adp8870_data.sub_led_blink_off_time)  == 0) {
			/*turn on led, the function is used for factory test*/
			adp8870_state_update(BLINK_ON);
			wake_unlock(&adp8870_data.wake_lock);
			hrtimer_cancel(&adp8870_data.sub_led_blink_timer);
			ADP8870_PRINTK(KERN_INFO"%s, brightness %d, cancel timer\n",
					__FUNCTION__,
					adp8870_data.sub_led_brighness);
			adp8870_set_sub_led_brightness(adp8870_data.sub_led_brighness >> 1);
		} else { /*turn off msg led since blink on or blink off time is 0*/
			adp8870_state_update(BLINK_OFF);
			wake_unlock(&adp8870_data.wake_lock);
			hrtimer_cancel(&adp8870_data.sub_led_blink_timer);
			adp8870_set_sub_led_brightness(0);
		}
	}
}


static int adp8870_sub_led_blink_set (/*struct led_classdev *led_cdev,*/
       int delay_on,  int delay_off)  /*delay_on and delay off are ms*/
{
     int sec = 0;
     int nsec = 0;

     sec = (delay_on >= MSEC_PER_SEC)?delay_on/MSEC_PER_SEC:0;
     nsec = (delay_on % MSEC_PER_SEC)* NSEC_PER_MSEC;        
     adp8870_data.sub_led_blink_on_time = ktime_set(sec, nsec);   
     ADP8870_PRINTK(KERN_INFO"%s,on time %d\n",__FUNCTION__, sec*MSEC_PER_SEC+nsec);
     sec = (delay_off >= MSEC_PER_SEC)?delay_off/MSEC_PER_SEC: 0;
     nsec = (delay_off % MSEC_PER_SEC)* NSEC_PER_MSEC;
     ADP8870_PRINTK(KERN_INFO"%s,off time %d\n",__FUNCTION__, sec*MSEC_PER_SEC+nsec);
     adp8870_data.sub_led_blink_off_time = ktime_set(sec, nsec);       

     if (adp8870_data.sub_led_brighness != LED_OFF) {        
        if (delay_on != 0 && delay_off != 0)  {
            adp8870_data.blink_state = STATE_ON;     
            /*start blink timer*/ 
            ADP8870_PRINTK(KERN_INFO"%s, start timer\n",__FUNCTION__);
            adp8870_state_update(BLINK_ON);
            wake_lock(&adp8870_data.wake_lock);
            hrtimer_start(&adp8870_data.sub_led_blink_timer, adp8870_data.sub_led_blink_on_time,HRTIMER_MODE_REL);
        }else if (delay_on == 0 && delay_off == 0){
            ADP8870_PRINTK(KERN_INFO"%s, cancel timer\n",__FUNCTION__);
	    wake_unlock(&adp8870_data.wake_lock);
            hrtimer_cancel(&adp8870_data.sub_led_blink_timer);
            adp8870_state_update(BLINK_ON);
            adp8870_set_sub_led_brightness(adp8870_data.sub_led_brighness >> 1);
        } else {
            ADP8870_PRINTK(KERN_INFO"%s,cancel timer\n",__FUNCTION__);
	    wake_unlock(&adp8870_data.wake_lock);
            hrtimer_cancel(&adp8870_data.sub_led_blink_timer);
            adp8870_state_update(BLINK_OFF);
            adp8870_set_sub_led_brightness(LED_OFF);
        }
     }
     return 0;
}



static void adp8870_set_sub_led_brightness(uint8_t brightness)
{
  mutex_lock(&adp8870_mutex);
  adp8870_write_reg(ADP8870_ISC7_REG, brightness, __FUNCTION__);
  adp8870_write_reg(ADP8870_ISCC_REG, ADP8870_BLSEN_1TO6_EN, __FUNCTION__);
  mutex_unlock(&adp8870_mutex);
}

static void adp8870_blink_func(struct work_struct *work)
{ 
    unsigned brightness = 0;
    
    
    UPDATE_BLINK_STATE(adp8870_data.blink_state);
    if ( adp8870_data.blink_state == STATE_ON)  {       
       brightness = adp8870_data.sub_led_brighness >> 1;   
       hrtimer_start(&adp8870_data.sub_led_blink_timer, adp8870_data.sub_led_blink_on_time,HRTIMER_MODE_REL);
    } else {
       hrtimer_start(&adp8870_data.sub_led_blink_timer, adp8870_data.sub_led_blink_off_time,HRTIMER_MODE_REL);       
    }
    adp8870_set_sub_led_brightness(brightness);    
}

static int adp8870_read_reg (unsigned reg, uint8_t *value)
{
   struct i2c_client *client = adp8870_data.client;
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

static int adp8870_write_reg (unsigned reg, uint8_t value, const char *caller)
{
	uint8_t buf[ADP8870_REG_WRITE_LEN] = { reg, value };
	int ret = 0;

	ADP8870_PRINTK ("%s: writing 0x%X to reg 0x%X at addr 0x%X\n",
			caller, buf[1], buf[0], adp8870_data.client->addr);

	ret = i2c_master_send (adp8870_data.client, buf, ADP8870_REG_WRITE_LEN);
	if (ret != ADP8870_REG_WRITE_LEN) {
		printk (KERN_ERR "%s: i2c_master_send error %d\n", caller, ret);
		if (ret >= 0) {
			/* Entire message wasn't written. Try again. */
			ret = -EAGAIN;
		}
	} else {
		ret = 0;
	}
	return ret;
}

static int adp8870_device_init(void)
{
	int ret = 0;

	ADP8870_PRINTK ("%s: Enter\n", __FUNCTION__);

	mutex_lock(&adp8870_mutex);
	/* disable the backlight dim timeout */
	ret = adp8870_write_reg(ADP8870_BLDIM_REG,
				ADP8870_BLDIM_NONE,
				__FUNCTION__);
	/* disable the backlight OFF timeout */
	if (!ret) {
		ret = adp8870_write_reg(ADP8870_BLOFF_REG,
					ADP8870_BLOFF_NONE,
					__FUNCTION__);
	}
	/* connect LEDs 1-6 to BL_EN */
	if (!ret) {
		adp8870_write_reg(ADP8870_BLSEN_REG,
					ADP8870_BLSEN_1TO6_EN,
					__FUNCTION__);
	}
	/* enable backlight in manual mode */
	if (!ret) {
		adp8870_write_reg(ADP8870_MDCR_REG,
					ADP8870_MDCR_MANUAL,
					__FUNCTION__);
	}

	if (ret) {
		printk (KERN_ERR "%s: failed\n", __FUNCTION__);
	}
	mutex_unlock(&adp8870_mutex);
	ADP8870_PRINTK ("%s: Exit, adp8870_state=%d, ret=%d\n",
			__FUNCTION__, adp8870_state, ret);

	return ret;
}

static void adp8870_set_brightness (struct led_classdev *led_cdev,
   enum led_brightness value, bool ramp)
{
   ADP8870_PRINTK ("%s: led_cdev->name=%s value=%d ramp=%d\n",
      __FUNCTION__, led_cdev->name, value, ramp);
   
   mutex_lock(&adp8870_mutex);

   if (value > LED_FULL)
   {
      value = LED_FULL;
   }  

   /* set brightness level */
   if (value >= BRIGHTNESS_MIN_SETTING)
       adp8870_data.user_setting_brightness = value;   
   adp8870_write_reg(ADP8870_BLMX1_REG, conv_setting_to_reg(value), __FUNCTION__);
   mutex_unlock(&adp8870_mutex);
   return;
}

static void adp8870_brightness_set (struct led_classdev *led_cdev,
   enum led_brightness value)
{

   if (value == LED_OFF) {
       adp8870_state_update(MAIN_LED_OFF);
   } else {
       adp8870_state_update(MAIN_LED_ON);
   }  
   adp8870_set_brightness(led_cdev, value, TRUE);
}

static void adp8870_brightness_set_nr (struct led_classdev *led_cdev,
   enum led_brightness value)
{
   adp8870_set_brightness(led_cdev, value, FALSE);
}

static enum hrtimer_restart adp8870_blink_timer_func(struct hrtimer *data)
{
    schedule_work(&adp8870_data.sub_led_blink_wq);
    return HRTIMER_NORESTART;
}

/* This function is called by i2c_probe */
static int adp8870_probe (struct i2c_client *client,
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

   adp8870_data.client = client;
   i2c_set_clientdata (client, &adp8870_data);

   /* Register LED class */
   ret = led_classdev_register (&client->adapter->dev, &adp8870_led);
   if (ret) {
      printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
         __FUNCTION__, adp8870_led.name, ret);
      return ret;
   } 
   ret = device_create_file (adp8870_led.dev, &dev_attr_als);
   ret = device_create_file (adp8870_led.dev, &dev_attr_pwm_mode);

   /* Register LED class for no ramping */
   ret = led_classdev_register (&client->adapter->dev, &adp8870_led_noramp);
   if (ret) {
      printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
         __FUNCTION__, adp8870_led.name, ret);
      return ret;
   } 

   ret = led_classdev_register (&client->adapter->dev, &adp8870_msg_led);
   if (ret) {
      printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
         __FUNCTION__, adp8870_msg_led.name, ret);
      return ret;
   } 
   
 
    ret = device_create_file (adp8870_msg_led.dev, &dev_attr_blink);
    dev_set_drvdata(adp8870_msg_led.dev, &adp8870_msg_led);
    wake_lock_init(&adp8870_data.wake_lock, WAKE_LOCK_SUSPEND, "msg_led");
#ifdef CONFIG_HAS_EARLYSUSPEND
   register_early_suspend (&early_suspend_data);
#endif

   /* TEMP HACK - Set brightness to 3/4 LED_FULL */
#ifndef TEMP_7XXX_CODE_COMPLETE
   adp8870_brightness_set_nr(&adp8870_led_noramp, (int)(LED_FULL * (3.0/4.0)));
#endif /* TEMP_7XXX_CODE_COMPLETE */

   printk ("%s: exit, ret=%d\n", __FUNCTION__, ret);
   adp8870_data.initialized = 1;
   hrtimer_init(&adp8870_data.sub_led_blink_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
   INIT_WORK (&adp8870_data.sub_led_blink_wq, adp8870_blink_func);
   adp8870_data.sub_led_blink_timer.function = adp8870_blink_timer_func;
   adp8870_state_update(MAIN_LED_ON);
   return 0;
}

static int adp8870_remove (struct i2c_client *client)
{
   led_classdev_unregister(&adp8870_led);
   led_classdev_unregister(&adp8870_led_noramp);
   return 0;
}

static int adp8870_suspend (struct i2c_client *client, pm_message_t mesg)
{

   adp8870_state_update(MAIN_LED_OFF);    
   ADP8870_PRINTK("%s: called with pm message %d\n", 
     __FUNCTION__, mesg.event);
   led_classdev_suspend (&adp8870_led);
   return 0;
}

static int adp8870_resume (struct i2c_client *client)
{
   
   printk("%s: resuming\n", __FUNCTION__);
   adp8870_state_update(MAIN_LED_ON); 
   led_classdev_resume (&adp8870_led);
   printk("%s: driver resumed\n", __FUNCTION__);
   return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp8870_early_suspend (struct early_suspend *h)
{
   adp8870_suspend (adp8870_data.client, PMSG_SUSPEND);
}

static void adp8870_late_resume (struct early_suspend *h)
{
   adp8870_resume (adp8870_data.client);
}
#endif



static int __devinit adp8870_driver_init (void)
{
   int ret;

   printk (KERN_INFO "%s: enter\n", __FUNCTION__);
   ret = i2c_add_driver (&adp8870_driver);
   if (ret) {
      printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n", 
         __FUNCTION__, ret);
   }

   return ret;
}

static void __exit adp8870_driver_exit(void)
{
   i2c_del_driver (&adp8870_driver);
}

module_init(adp8870_driver_init);
module_exit(adp8870_driver_exit);

MODULE_DESCRIPTION("ADP8870 DISPLAY BACKLIGHT DRIVER");
MODULE_AUTHOR("Fei Huang <fei.huang@motorola.com>")
MODULE_LICENSE("GPL v2");

