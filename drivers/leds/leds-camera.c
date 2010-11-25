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

// Linux driver for Camera flash LED

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <asm/gpio.h>
#include <mach/pmic.h>
#include <mach/vboost.h>
#include <mach/msm_rpcrouter.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


//--------------------------------------------------------------------------------
// 
// Literal definitions
// 
//--------------------------------------------------------------------------------

// GPIO input to Flash/GSM PA arbitration (controls FLSH_DRV_N via PMIC MPP 22)
#define MOTUS_CAMERA_LED_GPIO     76
#define MOTUS_LED_GPIO_OFF        0
#define MOTUS_LED_GPIO_ON         1
#define MOTUS_LED_GPIO_PULL       GPIO_NO_PULL

// GPIO input to strobe trigger circuit (controls FLSH_DRV_N via PMIC MPP 22)
#define ZEPPELIN_CAMERA_LED_GPIO  108
#define ZEPPELIN_LED_GPIO_OFF     0 
#define ZEPPELIN_LED_GPIO_ON      1 
#define ZEPPELIN_LED_GPIO_PULL    GPIO_PULL_DOWN

// PMIC LCD output current granularity (in mA)
#define GRANULARITY_PMIC_LCD_CURRENT 10

// PMIC LCD output current upper limit (in mA)
#define MAX_PMIC_LCD_CURRENT 150

// PMIC flash output current granularity (in mA)
#define GRANULARITY_PMIC_FLASH_CURRENT 40

// PMIC flash burst LED output current upper limit (in mA)
#define MAX_PMIC_BURST_CURRENT 350

// flash burst LED output time upper limit (in msec)
#define MAX_FLASH_LED_BURST_DURATION 500

// flash burst LED output time lower limit (in msec)
#define MIN_FLASH_LED_BURST_DURATION 150

// flash burst LED brightness lower limit
#define MIN_FLASH_LED_BURST_BRIGHTNESS 100

// LED torch/flashlight Current upper limit, per LED spec (in mA)
#define MAX_FLASH_LED_TORCH_CURRENT 100

// number of torch/flashlight levels (in PMIC) within spec for this LED
#define LED_TORCH_LEVELS (MAX_FLASH_LED_TORCH_CURRENT/GRANULARITY_PMIC_LCD_CURRENT)

// flash torch LED brightness lower limit for illumination
#define MIN_FLASH_LED_TORCH_BRIGHTNESS (256 / LED_TORCH_LEVELS)


//--------------------------------------------------------------------------------
// 
// Local function prototypes
// 
//--------------------------------------------------------------------------------
static void cam_led_work_func(struct work_struct *work);
static enum hrtimer_restart cam_led_timer_func(struct hrtimer *cam_led_timer);

static int cam_led_do_torch_level(unsigned value);
static int cam_led_do_burst_level(unsigned value);

static void cam_led_brightness_set(struct led_classdev *cdev, unsigned val);
static void cam_led_torch_brightness_set(struct led_classdev *cdev, unsigned val);
static void cam_led_burst_brightness_set(struct led_classdev *cdev, unsigned val);

static int cam_led_suspend(struct platform_device *pdev, pm_message_t mesg);
static int cam_led_resume(struct platform_device *dev);
#ifdef CONFIG_ANDROID_POWER
static void cam_led_early_suspend(struct android_early_suspend *h);
static void cam_led_late_resume(struct android_early_suspend *h);
#endif

static ssize_t cam_led_burst_duration_show(struct device *dev,
                                           struct device_attribute *attr, 
                                           char *buf);
static ssize_t cam_led_burst_duration_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t size);
static ssize_t cam_led_burst_brightness_show(struct device *dev,
                                             struct device_attribute *attr, 
                                             char *buf);
static ssize_t cam_led_burst_brightness_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf, size_t size);
static ssize_t cam_led_torch_brightness_show(struct device *dev,
                                             struct device_attribute *attr, 
                                             char *buf);
static ssize_t cam_led_torch_brightness_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf, size_t size);

static int cam_led_probe(struct platform_device *pdev);
static int cam_led_remove(struct platform_device *pdev);
static int cam_led_register(struct device *dev);
static void cam_led_unregister(void);
static void cam_led_dev_release(struct device *dev);
static int __devinit cam_led_init(void);
static void __exit cam_led_exit(void);
static void cam_led_remove_device_files(void);
static int cam_led_create_device_files(void);

//--------------------------------------------------------------------------------
// 
// Exported prototypes for setting Camera LED attributes
// 
//--------------------------------------------------------------------------------
int cam_led_set_burst_brightness(unsigned value);
int cam_led_get_burst_brightness(unsigned *value);
int cam_led_set_torch_brightness(unsigned value);
int cam_led_get_torch_brightness(unsigned *value);
int cam_led_set_burst_duration(unsigned value);
int cam_led_get_burst_duration(unsigned *value);

EXPORT_SYMBOL(cam_led_set_burst_brightness);
EXPORT_SYMBOL(cam_led_get_burst_brightness);
EXPORT_SYMBOL(cam_led_set_torch_brightness);
EXPORT_SYMBOL(cam_led_get_torch_brightness);
EXPORT_SYMBOL(cam_led_set_burst_duration);
EXPORT_SYMBOL(cam_led_get_burst_duration);

//--------------------------------------------------------------------------------
// 
// Exported prototypes for controlling Cameral LED state
// 
//--------------------------------------------------------------------------------
int cam_led_burst(void);
int cam_led_torch(void);
int cam_led_timed_torch(unsigned msec);
int cam_led_off(void);
int cam_led_brightness(unsigned value);

EXPORT_SYMBOL(cam_led_burst);
EXPORT_SYMBOL(cam_led_torch);
EXPORT_SYMBOL(cam_led_timed_torch);
EXPORT_SYMBOL(cam_led_off);
EXPORT_SYMBOL(cam_led_brightness);


//--------------------------------------------------------------------------------
// 
// Local/static data declarations
// 
//--------------------------------------------------------------------------------
// Current (now) brightness level for the flash LED
static int curr_brightness = 0;

// Default brightness level (stored attribute) in torch mode
static int torch_brightness = 255;  // Equates to 100 mA (tuning value)

// Default brightness level (stored attribute) in burst mode
static int burst_brightness = 255;  // Equates to 400 mA (a safe value)

// Default duration (stored attribute) of burst mode (in msec)
static int burst_duration = 500;

// GPIO that will be used to control the Camera LED burst level
static int camera_led_gpio;

// On & Off values for camera_led_gpio (initialized to opposite sense)
static int led_gpio_off;
static int led_gpio_on;

static struct platform_device cam_led_device = {
    .name = "camera-led",
    .dev  = {
        .release = cam_led_dev_release,
    }
};

// Camera LED Driver entry points
static struct platform_driver cam_led_driver = {
    .probe   = cam_led_probe,
    .remove  = cam_led_remove,
#ifdef CONFIG_PM
    .suspend = cam_led_suspend,
    .resume  = cam_led_resume,
#endif
    .driver = {
        .name = "camera-led",
        .owner = THIS_MODULE,
    },
};

// Camera LED logical device brightness functions
static struct led_classdev cam_leds[] = {
    {
        .name = "cam-burst",
        .brightness_set = cam_led_burst_brightness_set,
    },
    {
        .name = "cam-torch",
        .brightness_set = cam_led_torch_brightness_set,
    },
    {
        .name = "flashlight",
        .brightness_set = cam_led_brightness_set,
    },
};
 
// Camera LED burst duration attribute functions
static struct device_attribute burst_duration_attr = {
        .attr = {
                 .name = "duration",
                 .mode = 0666,
        },
        .show = cam_led_burst_duration_show,
        .store = cam_led_burst_duration_store,
};

// Camera LED burst brightness attribute functions
static struct device_attribute burst_brightness_attr = {
        .attr = {
                 .name = "brightness-setting",
                 .mode = 0666,
        },
        .show = cam_led_burst_brightness_show,
        .store = cam_led_burst_brightness_store,
};

// Camera LED torch brightness attribute functions
static struct device_attribute torch_brightness_attr = {
        .attr = {
                 .name = "brightness-setting",
                 .mode = 0666,
        },
        .show = cam_led_torch_brightness_show,
        .store = cam_led_torch_brightness_store,
};

#ifdef CONFIG_ANDROID_POWER

// For tracking suspend & resume
struct cam_led {
    struct flash_client *client;
};

static struct cam_led cam_led_data;

static struct led_classdev suspend_cam_led = {
    .name = "brightness",
    .brightness_set = cam_led_brightness_set
};

static struct android_early_suspend early_suspend = {
    .level = ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
    .suspend = cam_led_early_suspend,
    .resume = cam_led_late_resume,
};
#endif

// For timed torch or burst (safety) time limit
static struct hrtimer cam_led_timer;
static struct work_struct cam_led_work;


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Camera LED brightness "work" functions
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_work_func()
//
//--------------------------------------------------------------------------- 
static void cam_led_work_func(struct work_struct *work)
{
    printk(KERN_INFO "%s: terminating any active torch/burst\n", __FUNCTION__);

    // the only work here is to end the pending torch/burst
    cam_led_off();
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_timer_func()
//
//--------------------------------------------------------------------------- 
static enum hrtimer_restart cam_led_timer_func(struct hrtimer *cam_led_timer)
{
    printk(KERN_INFO "%s: scheduling work function...\n", __FUNCTION__);

    schedule_work(&cam_led_work);
    return HRTIMER_NORESTART;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_do_torch_level()
// 
//--------------------------------------------------------------------------- 
static int cam_led_do_torch_level(unsigned value)
{
    // translate brightness (0-255) into the permissible intensity level
    int level = ((value & 255) * LED_TORCH_LEVELS) / 255;
    int ret = 0;

    // ensure this machine supports an LED flash
    if (!machine_is_motus() && !machine_is_zeppelin()) {
        printk(KERN_ERR "%s: NOT Motus or Zeppelin; aborting.\n", __FUNCTION__);
        return -EINVAL;
    }
    if (value == 0) {
         // Disable VBoost VREG to minimize current drain when LED is OFF
        vboost_disable(VBOOST_CAMERA);
    } else {
        // All Camera LED usage requires the VBoost VREG enabled
        vboost_enable(VBOOST_CAMERA);
    }
   // set the current level in the PMIC LCD output used for torch/flashlight
    printk("cam_led_do_torch_level : Intensity: %d , level: %d \n ", value, level); 
    ret = set_led_intensity(LED_LCD, level);
    if (ret) {
        printk(KERN_ERR "%s: set_led_intensity() failed, error %d\n",
               __FUNCTION__, ret);
    } else {
        // update the current brightness level
        curr_brightness = (value & 255);
    }
    return ret;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_do_burst_level()
// 
//--------------------------------------------------------------------------- 
static int cam_led_do_burst_level(unsigned value)
{
    // Common work function for burst: controls low & high current sources
    int high_curr, low_curr, total_curr, adj_curr, level = 0;
    int ret = 0;

    // ensure this machine supports an LED flash
    if (!machine_is_motus() && !machine_is_zeppelin()) {
        printk(KERN_ERR "%s: NOT Motus or Zeppelin; aborting.\n", __FUNCTION__);
        return -EINVAL;
    }
    // First check for the "off" state transition
    if (value == 0) {
        if (machine_is_motus()) { 
            // Set the Camera LED GPIO to the "off" state (do before RPCs)
            gpio_set_value(camera_led_gpio, led_gpio_off);
        } 
        // Disable VBoost VREG to minimize current drain
        vboost_disable(VBOOST_CAMERA);

        // Disable the flash LED burst current level in the PMIC
        ret |= flash_led_set_current(0);
        if (ret) {
            printk(KERN_ERR "%s: flash_led_set_current() off failed, error %d\n",
                   __FUNCTION__, ret);
            // Re-attempt to disable the flash LED burst current in the PMIC
            ret |= flash_led_set_current(0);
            if (ret) {
                printk(KERN_ERR "%s: flash_led_set_current() re-try failed, error %d\n", 
                       __FUNCTION__, ret);
            }
        }
        // Disable the flash LED torch current in the PMIC
        ret = set_led_intensity(LED_LCD, 0);
        if (ret) {
            printk(KERN_ERR "%s: set_led_intensity() off failed, error %d\n",
                   __FUNCTION__, ret);
 
            // Re-attempt to disable the flash LED torch current in the PMIC
            ret = set_led_intensity(LED_LCD, 0);
            if (ret) {
                printk(KERN_ERR "%s: set_led_intensity() re-try failed, error %d\n",
                       __FUNCTION__, ret);
            }
        }
        return ret;
    } 
    // All Camera LED usage requires the VBoost VREG enabled
    vboost_enable(VBOOST_CAMERA);

    // Convert brightness (0-255) to total current level (0-500 mA)
    total_curr = ((MAX_PMIC_BURST_CURRENT + MAX_PMIC_LCD_CURRENT)
               * (value & 255)) / 255;

    // Adjust the total current to the finer granularity
    total_curr -= total_curr % GRANULARITY_PMIC_LCD_CURRENT;

    // Partition the total current between the two current sources
    if (total_curr <= MAX_PMIC_LCD_CURRENT) {
        // high current source is not needed
        high_curr = 0;

        // translate brightness (0-255) into intensity level (0-15)
        level = ((value & 255) * 15) / 255;
    } else {
        // compute an ideal partitioning (maximizing low current source)
        low_curr  = MAX_PMIC_LCD_CURRENT;
        high_curr = total_curr - low_curr;

        // Adjust current sources, considering the coarse granularity
        adj_curr = high_curr % GRANULARITY_PMIC_FLASH_CURRENT;
        if (adj_curr)
        {
            high_curr += GRANULARITY_PMIC_FLASH_CURRENT - adj_curr;
            low_curr  -= GRANULARITY_PMIC_FLASH_CURRENT - adj_curr;
        } 
        // translate low current (0-150) into intensity level (0-15)
        level = low_curr / 10;
    }
    printk("%s : total_curr: %d, high_curr: %d, low_curr: %d, torch level: %d\n",
			__FUNCTION__, total_curr, high_curr, low_curr, level);
 
    // Update the flash LED burst current level in the PMIC
    ret |= flash_led_set_current(high_curr);
    if (ret) {
        printk(KERN_ERR "%s: cam_led_set_current() failed, error %d\n",
               __FUNCTION__, ret);
    }
    // Enable the flash LED torch current in the PMIC
    ret = set_led_intensity(LED_LCD, level);
    if (ret) {
        printk(KERN_ERR "%s: set_led_intensity() on failed, error %d\n",
               __FUNCTION__, ret);
    }
    if (machine_is_zeppelin()) {
        // Toggle the Camera LED GPIO, forming a 1 msec trigger pulse for 
        // the strobe circuit.  This circuit will automatically lower the
        // GPIO when the timeout period has expired.  In the meantime, if
        // this function is called with a value of 0 (to terminate the burst) 
        // the two output current levels will disabled in the PMIC and the 
        // LED will turn off as expected.
        gpio_set_value(camera_led_gpio, led_gpio_on);
        udelay(1000);
        gpio_set_value(camera_led_gpio, led_gpio_off);
    } else {
        // Set the Camera LED GPIO to the "on" state.  This GPIO will be 
        // returned to the "off" state later, when this function is called
        // with a value of 0 (or when the HW or SW safety timer expires).
        gpio_set_value(camera_led_gpio, led_gpio_on);
    }
    // no need to update curr_brightness value -- back to zero in a "flash"
    return ret;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Camera LED logical device brightness functions
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_brightness_set()
// 
//--------------------------------------------------------------------------- 
static void cam_led_brightness_set(struct led_classdev *led_cdev, 
                                   unsigned value)
{
    // Do an un-timed illumination (aka flashlight mode)
    int rc = 0;

    printk(KERN_INFO "%s: %s %d\n", __FUNCTION__, led_cdev->name, value);

    // disarm any active torch or burst timer
    hrtimer_cancel(&cam_led_timer);

    // use the low current source work function (same as torch)
    rc = cam_led_do_torch_level(value);
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_torch_brightness_set()
// 
//--------------------------------------------------------------------------- 
static void cam_led_torch_brightness_set(struct led_classdev *led_cdev,
                                         unsigned value)
{
    int rc = 0;

    // Do an un-timed "torch" mode illumination 
    printk(KERN_INFO "%s: %s %d\n", __FUNCTION__, led_cdev->name, value);

    // disarm any active torch or burst timer
    hrtimer_cancel(&cam_led_timer);

    // use the low current source work function
    rc = cam_led_do_torch_level(value);
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_burst_brightness_set()
//
//--------------------------------------------------------------------------- 
static void cam_led_burst_brightness_set(struct led_classdev *led_cdev,
                                         unsigned value)
{
    // Do a timed burst at the specified brightness (no initial torch)
    int rc = 0;

    printk(KERN_INFO "%s: %s %d\n", __FUNCTION__, led_cdev->name, value);

    // Turn the burst on
    rc = cam_led_do_burst_level(value);

    // Delay for the interval tracked by the "duration" attrib (msec)
    mdelay(burst_duration);

    // Turn the burst off
    rc |= cam_led_do_burst_level(0);
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Camera LED sysfs attribute "show & store" functions
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_burst_duration_show()
//
//--------------------------------------------------------------------------- 
static ssize_t cam_led_burst_duration_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    sprintf(buf, "%d mSec\n", burst_duration);
    return strlen(buf) + 1;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_burst_duration_store()
//
//--------------------------------------------------------------------------- 
static ssize_t cam_led_burst_duration_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t size)
{
    int duration, n = 0;

    if (!buf || size == 0) {
        printk(KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    n = sscanf(buf, "%d", &duration);
    if (n != 1) {
        printk(KERN_ERR "%s: invalid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    if (duration < MIN_FLASH_LED_BURST_DURATION || 
        duration > MAX_FLASH_LED_BURST_DURATION) {
        printk(KERN_ERR "%s: invalid duration %d, must be %d to %d msec\n",
               __FUNCTION__, duration, MIN_FLASH_LED_BURST_DURATION,
               MAX_FLASH_LED_BURST_DURATION);
        return -EINVAL;
    }
    printk(KERN_ERR "%s: duration = %d\n", __FUNCTION__, duration);

    // save the desired duration time for the flash burst
    burst_duration = duration;
    return size;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_burst_brightness_show()
//
//--------------------------------------------------------------------------- 
static ssize_t cam_led_burst_brightness_show(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf)
{
    sprintf(buf, "%d\n", burst_brightness);
    return strlen(buf) + 1;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_burst_brightness_store()
//
//--------------------------------------------------------------------------- 
static ssize_t cam_led_burst_brightness_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf, size_t size)
{
    int bright, n = 0;

    if (!buf || size == 0) {
        printk(KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    n = sscanf (buf, "%d", &bright);
    if (n != 1) {
        printk(KERN_ERR "%s: invalid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    // Adjust to minimum granularity so "on" is never "off"
    if (bright < MIN_FLASH_LED_BURST_BRIGHTNESS) {
        printk(KERN_ERR "%s: brightness %d adjusted to min (%d)\n",
               __FUNCTION__, bright, MIN_FLASH_LED_BURST_BRIGHTNESS);
        bright = MIN_FLASH_LED_BURST_BRIGHTNESS;
    }
    else if (bright > 255) {
        printk(KERN_ERR "%s: brightness %d adjusted to max (255)\n",
               __FUNCTION__, bright);
        bright = 255;
    }
    printk(KERN_ERR "%s: cam-burst/brightness-setting = %d\n", 
           __FUNCTION__, bright);

    // save the burst brightness level
    burst_brightness = bright;
    return size;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_torch_brightness_show()
//
//--------------------------------------------------------------------------- 
static ssize_t cam_led_torch_brightness_show(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf)
{
    sprintf(buf, "%d\n", torch_brightness);
    return strlen(buf) + 1;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_torch_brightness_store()
//
//--------------------------------------------------------------------------- 
static ssize_t cam_led_torch_brightness_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf, size_t size)
{
    int bright, n = 0;

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }
    n = sscanf(buf, "%d", &bright);
    if (n != 1) {
        printk(KERN_ERR "%s: invalid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    if (bright < 1 || bright > 255) {
        printk(KERN_ERR "%s: invalid brightness %d, must be 1-255\n",
               __FUNCTION__, bright);
        return -EINVAL;
    }
    printk(KERN_ERR "%s: cam-torch/brightness-setting = %d\n",
           __FUNCTION__, bright);

    // save the desired brightness level for torch mode
    torch_brightness = bright;
    return size;
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Camera LED driver kernel space interface functions to Camera Driver
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_get_burst_duration()
//
//--------------------------------------------------------------------------- 
int cam_led_get_burst_duration(unsigned *value)
{
    if (value == NULL) {
        return -EFAULT;
    }
    *value = burst_duration;
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_set_burst_duration()
//
//--------------------------------------------------------------------------- 
int cam_led_set_burst_duration(unsigned value)
{
    if (value < MIN_FLASH_LED_BURST_DURATION || 
        value > MAX_FLASH_LED_BURST_DURATION) {
        printk(KERN_ERR "%s: duration %d out of range (%d-%d)\n", 
               __FUNCTION__, value, MIN_FLASH_LED_BURST_DURATION, 
               MIN_FLASH_LED_BURST_DURATION);
        return -EFAULT;
    }
    printk(KERN_ERR "%s: duration = %d\n", __FUNCTION__, value);

    // save the desired duration time for the flash burst
    burst_duration = value;
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_get_burst_brightness()
//
//--------------------------------------------------------------------------- 
int cam_led_get_burst_brightness(unsigned *value)
{
    if (value == NULL) {
        return -EFAULT;
    }
    *value = burst_brightness;
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_set_burst_brightness()
//
//--------------------------------------------------------------------------- 
int cam_led_set_burst_brightness(unsigned value)
{
    if (value < 1 || value > 255) {
        printk(KERN_ERR "%s: brightness %d out of range (1-255)\n", 
               __FUNCTION__, value);
        return -EFAULT;
    }
    printk(KERN_ERR "%s: brightness = %d\n", __FUNCTION__, value);

    // Adjust to minimum granularity so "on" is never "off"
    if (value < MIN_FLASH_LED_BURST_BRIGHTNESS) {
        printk(KERN_ERR "%s: brightness %d adjusted to min (%d)\n", 
               __FUNCTION__, value, MIN_FLASH_LED_BURST_BRIGHTNESS);
	value = MIN_FLASH_LED_BURST_BRIGHTNESS;
    }
    // save the desired brightness level for the flash burst
    burst_brightness = value;
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_get_torch_brightness()
//
//--------------------------------------------------------------------------- 
int cam_led_get_torch_brightness(unsigned *value)
{
    if (value == NULL) {
        return -EFAULT;
    }
    *value = torch_brightness;
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_set_torch_brightness()
//
//--------------------------------------------------------------------------- 
int cam_led_set_torch_brightness(unsigned value)
{
    if (value < 1 || value > 255) {
        printk(KERN_ERR "%s: brightness %d out of range (1-255)\n", 
               __FUNCTION__, value);
        return -EFAULT;
    }
    printk(KERN_ERR "%s: brightness = %d\n", __FUNCTION__, value);

    // Adjust to minimum granularity so "on" is never "off"
    if (value < MIN_FLASH_LED_TORCH_BRIGHTNESS) {
        printk(KERN_ERR "%s: brightness %d adjusted to min (%d)\n", 
               __FUNCTION__, value, MIN_FLASH_LED_TORCH_BRIGHTNESS);
	value = MIN_FLASH_LED_TORCH_BRIGHTNESS;
    }
    // save the desired torch brightness level
    torch_brightness = value;
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_burst()
//
//--------------------------------------------------------------------------- 
int cam_led_burst(void)
{
    // Initiate a high-current pulse, based on the burst brightness attribute.
    int rc = 0;
    unsigned msec = MAX_FLASH_LED_BURST_DURATION;

    printk(KERN_INFO "%s: brightness %d\n", __FUNCTION__, burst_brightness);

    // Disarm any active torch or burst timer 
    hrtimer_cancel(&cam_led_timer);

    // Turn the burst on
    rc = cam_led_do_burst_level(burst_brightness);

    // Initiate the safety timer count-down
    hrtimer_start(&cam_led_timer, 
                  ktime_set(msec / 1000, (msec % 1000) * 1000000), 
                  HRTIMER_MODE_REL);

    printk(KERN_INFO "%s: exiting\n", __FUNCTION__);
    return rc;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_torch()
//
//--------------------------------------------------------------------------- 
int cam_led_torch(void)
{
    // Do an un-timed, low-current illumination, based on the saved torch
    // brightness attribute.
 
    int rc = 0;

    printk(KERN_INFO "%s: brightness %d\n", __FUNCTION__, torch_brightness);

    // disarm any active torch or burst timer 
    hrtimer_cancel(&cam_led_timer);

    // start torching via the low current source, until cam_led_off() called
    rc = cam_led_do_torch_level(torch_brightness);

    printk(KERN_INFO "%s: exiting\n", __FUNCTION__);
    return rc;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_timed_torch()
//
//--------------------------------------------------------------------------- 
int cam_led_timed_torch(unsigned msec)
{
    // Do an timed, low-current illumination, based on the saved torch
    // brightness attribute.
 
    int rc = 0;

    printk(KERN_INFO "%s: duration %d msec\n", __FUNCTION__, msec);

    // disarm any active torch or burst timer 
    hrtimer_cancel(&cam_led_timer);

    // start torching via the low current source, until the timer expires
    rc = cam_led_do_torch_level(torch_brightness);

    // initiate the timer count-down for this variety of torch mode
    hrtimer_start(&cam_led_timer, 
                  ktime_set(msec / 1000, (msec % 1000) * 1000000), 
                  HRTIMER_MODE_REL);

    printk(KERN_INFO "%s: exiting\n", __FUNCTION__);
    return rc;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_off()
//
//--------------------------------------------------------------------------- 
int cam_led_off(void)
{
    // Turn the Camera LED off.  Using the high current source work
    // function ensures that both output sources will be disabled.

    int rc = 0;

    printk(KERN_INFO "%s: brightness set to zero\n", __FUNCTION__);

    // disarm any active torch or burst timer 
    hrtimer_cancel(&cam_led_timer);

    // use the high current source work function 
    rc = cam_led_do_burst_level(0);

    printk(KERN_INFO "%s: exiting\n", __FUNCTION__);
    return rc;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_brightness()
//
//--------------------------------------------------------------------------- 
int cam_led_brightness(unsigned value)
{
    // Do an un-timed, low-current illumination based on the specified
    // brightness input argument.
 
    int rc = 0;

    printk(KERN_INFO "%s: brightness %d\n", __FUNCTION__, value);

    // disarm any active torch or burst timer 
    hrtimer_cancel(&cam_led_timer);

    // use the low current source work function
    rc = cam_led_do_torch_level(value);

    printk(KERN_INFO "%s: exiting\n", __FUNCTION__);
    return rc;
}

#ifdef CONFIG_PM
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Camera LED class diver power management functions
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_suspend()
//
//--------------------------------------------------------------------------- 
static int cam_led_suspend (struct platform_device *pdev, pm_message_t mesg)
{
    int ret = 0;

    printk(KERN_INFO "%s enter: %d\n", __FUNCTION__, mesg.event);

    // ensure this machine supports an LED flash
    if (!machine_is_motus() && !machine_is_zeppelin()) {
        printk(KERN_ERR "%s: NOT Motus or Zeppelin; aborting.\n", __FUNCTION__);
        return -EINVAL;
    }
    // set the current level in the LCD output to zero (burst is transient)
    ret = set_led_intensity(LED_LCD, 0);
    if (ret) {
        printk (KERN_ERR "%s: set_led_intensity(0) failed, error %d\n",
                 __FUNCTION__, ret);
        return ret;
    }
    // update the current brightness level
    curr_brightness = 0;

    printk(KERN_INFO "%s exit: LED intensity set to zero.\n", __FUNCTION__);
    return ret;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_resume()
//
//--------------------------------------------------------------------------- 
static int cam_led_resume(struct platform_device *dev)
{
    printk(KERN_INFO "%s enter\n", __FUNCTION__);
    // In any mode, the Camera LED should not be re-illuminated 
    return 0;    
}

#ifdef CONFIG_ANDROID_POWER
//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_early_suspend()
//
//--------------------------------------------------------------------------- 
static void cam_led_early_suspend(struct android_early_suspend *h)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    cam_led_suspend (cam_led_data.client, PMSG_SUSPEND);
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_late_resume()
//
//--------------------------------------------------------------------------- 
static void cam_led_late_resume(struct android_early_suspend *h)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    cam_led_resume(cam_led_data.client);
}
#endif
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Camera LED class driver initialization functions
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_probe()
//
//--------------------------------------------------------------------------- 
static int cam_led_probe(struct platform_device *pdev)
{
    int ret = 0;
    int led_gpio_pull;


    printk(KERN_INFO "%s: enter\n", __FUNCTION__);

    if (machine_is_motus()) {
        camera_led_gpio = MOTUS_CAMERA_LED_GPIO;
        led_gpio_off    = MOTUS_LED_GPIO_OFF;
        led_gpio_on     = MOTUS_LED_GPIO_ON;
        led_gpio_pull   = MOTUS_LED_GPIO_PULL;
    } else if (machine_is_zeppelin()) {
        camera_led_gpio = ZEPPELIN_CAMERA_LED_GPIO;
        led_gpio_off    = ZEPPELIN_LED_GPIO_OFF;
        led_gpio_on     = ZEPPELIN_LED_GPIO_ON;
        led_gpio_pull   = ZEPPELIN_LED_GPIO_PULL;
    } else {
        printk(KERN_ERR "%s: NOT Motus or Zeppelin; aborting.\n", __FUNCTION__);
        return -EINVAL;
    }
    // Request ownership of the GPIO for the Camera flash LED
    ret = gpio_request(camera_led_gpio, "cam_led_ctrl");
    if (ret) {
        printk(KERN_ERR "%s: gpio_request(%d, cam_led_ctrl) error %d\n",
               __FUNCTION__, camera_led_gpio, ret);
        return ret;
    }
    // Set the Top Level Muxing attributes for this GPIO
    gpio_tlmm_config(GPIO_CFG(camera_led_gpio, 0, GPIO_OUTPUT, 
                              led_gpio_pull, GPIO_2MA), GPIO_ENABLE);

    // Assign the GPIO as an output line
    ret = gpio_direction_output(camera_led_gpio, 1);
    if (ret) {
        printk(KERN_ERR "%s: gpio_direction_output(%d, 1) error %d\n",
               __FUNCTION__, camera_led_gpio, ret);
        gpio_free(camera_led_gpio);
        return ret;
    }
    // Set the initial value of this GPIO to the "off" state
    gpio_set_value(camera_led_gpio, led_gpio_off);
 
    // Register the Camera LED Class driver with the kernel
    ret = cam_led_register(&pdev->dev);
    if (ret) {
        gpio_free(camera_led_gpio);
        return ret;
    }

#ifdef CONFIG_ANDROID_POWER
    android_register_early_suspend(&early_suspend);
#endif

    // Create the additional device attribute files in sysfs
    ret = cam_led_create_device_files();

    // Create high-res timer for timed torch or burst safety limit
    hrtimer_init(&cam_led_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    cam_led_timer.function = cam_led_timer_func;
    INIT_WORK(&cam_led_work, cam_led_work_func);

    // Ensure that both LED current sources are in the "off" state
    ret = cam_led_off();
    if (ret) {
        printk (KERN_ERR "%s: Failed to disable PMIC output(s) !, error %d\n",
                __FUNCTION__, ret);
    }
    printk (KERN_ERR "%s: exit\n", __FUNCTION__);
    return ret;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_remove()
//
//--------------------------------------------------------------------------- 
static int cam_led_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "%s: enter\n", __FUNCTION__);

    cam_led_unregister();
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_unregister()
//
//--------------------------------------------------------------------------- 
static void cam_led_unregister(void)
{
    int i;
    int attrs = sizeof (cam_leds) / sizeof (cam_leds[0]);

    for (i = 0; i < attrs; i++) {
        led_classdev_unregister(&cam_leds[i]);
    }
    gpio_free(camera_led_gpio);
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_register()
//
//--------------------------------------------------------------------------- 
static int cam_led_register(struct device *dev)
{
    int i, ret;
    int leds = sizeof (cam_leds) / sizeof (cam_leds[0]);

    for (i = 0; i < leds; i++) {
        ret = led_classdev_register(dev, &cam_leds[i]);
        if (ret) {
            printk (KERN_ERR "%s: unable to register %s LED: %d\n",
                __FUNCTION__, cam_leds[i].name, ret);
            cam_led_unregister();
            return ret;
        }
    }
    return 0;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_dev_release()
//
//--------------------------------------------------------------------------- 
static void cam_led_dev_release(struct device *dev)
{      
        gpio_free(camera_led_gpio);
}


//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_remove_device_files()
//
//--------------------------------------------------------------------------- 
static void cam_led_remove_device_files(void)
{
    device_remove_file(cam_leds[0].dev, &burst_duration_attr);
    device_remove_file(cam_leds[0].dev, &burst_brightness_attr);
    device_remove_file(cam_leds[1].dev, &torch_brightness_attr);
}


//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_create_device_files()
//
//--------------------------------------------------------------------------- 
static int cam_led_create_device_files(void)
{
    int ret;

    ret = device_create_file(cam_leds[0].dev, &burst_duration_attr);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"duration\" file for %s: %d\n",
                __FUNCTION__, cam_leds[0].name, ret);
       cam_led_remove_device_files();
       return ret;
    }
    dev_set_drvdata(cam_leds[0].dev, &cam_leds[0]);
 
    ret = device_create_file(cam_leds[0].dev, &burst_brightness_attr);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"setting\" file for %s: %d\n",
                __FUNCTION__, cam_leds[0].name, ret);
        cam_led_remove_device_files();
        return ret;
    }
    ret = device_create_file(cam_leds[1].dev, &torch_brightness_attr);
    if (ret) {
        printk (KERN_ERR "%s: unable to create \"setting\" file for %s: %d\n",
                __FUNCTION__, cam_leds[1].name, ret);
        cam_led_remove_device_files();
        return ret;
    }
    return 0;
}


//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_init()
//
//--------------------------------------------------------------------------- 
static int __init cam_led_init(void)
{
    int ret = 0;

    printk (KERN_ERR "%s: enter\n", __FUNCTION__);

    // Register this driver's entry points
    ret = platform_driver_register(&cam_led_driver);
    if (ret) {
        printk (KERN_ERR "%s: platform_driver_register returned %d\n",
            __FUNCTION__, ret);
        return ret;
    }
    // Register the "camera-led" device
    ret = platform_device_register(&cam_led_device);
    if (ret) {
        printk (KERN_ERR "%s: platform_device_register returned %d\n",
            __FUNCTION__, ret);
        return ret;
    }
    printk (KERN_ERR "%s: exit\n", __FUNCTION__);
    return ret;
}

//--------------------------------------------------------------------------- 
// 
// FUNCTION: cam_led_exit()
//
//--------------------------------------------------------------------------- 
static void __exit cam_led_exit(void)
{
    printk (KERN_ERR "%s: enter\n", __FUNCTION__);
    platform_device_unregister(&cam_led_device);
    platform_driver_unregister(&cam_led_driver);
}

module_init(cam_led_init);
module_exit(cam_led_exit);

MODULE_AUTHOR("MGD <noreply@motorola.com>");
MODULE_DESCRIPTION("CAMERA LED DRIVER");
MODULE_LICENSE("GPL v2");
