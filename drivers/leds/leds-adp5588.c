/*
 * leds-adp5588.c - ADP5588 Qwerty Keyboard LED driver
 *
 * Copyright (c) 2009 Motorola, INC.
 * Author: Vinh Vo <vinh@motorola.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp5588_early_suspend (struct early_suspend *h);
static void adp5588_late_resume (struct early_suspend *h);
static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2,
    .suspend = adp5588_early_suspend,
    .resume = adp5588_late_resume,
};

#endif

extern int adp5588_set_backlight(uint8_t mask);

static void 
adp5588_keypad_bl_led_set(struct led_classdev *led_cdev,
                      enum led_brightness value)
{
	if (adp5588_set_backlight(value) < 0)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct 
led_classdev adp5588_kp_bl_led = {
   .name            = "keyboard-backlight",
   .brightness_set  = adp5588_keypad_bl_led_set,
	.brightness	     = LED_OFF,
};

static int 
adp5588_led_probe(struct platform_device *pdev)
{
        int ret = 0;

	ret = led_classdev_register(&pdev->dev, &adp5588_kp_bl_led);

#ifdef CONFIG_HAS_EARLYSUSPEND
   if (!ret)
      register_early_suspend (&early_suspend_data);
#endif

	return ret;
}

static int __devexit 
adp5588_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&adp5588_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int 
adp5588_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&adp5588_kp_bl_led);

	return 0;
}

static int 
adp5588_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&adp5588_kp_bl_led);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp5588_early_suspend (struct early_suspend *h)
{
    adp5588_led_suspend (NULL, PMSG_SUSPEND);
}

static void adp5588_late_resume (struct early_suspend *h)
{
    adp5588_led_resume (NULL);
}
#endif
#else
#define adp5588_led_suspend  NULL
#define adp5588_led_resume   NULL
#endif

static struct platform_driver 
adp5588_led_driver = {
   .probe      = adp5588_led_probe,
   .remove     = __devexit_p(adp5588_led_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
   .suspend	   = adp5588_led_suspend,
   .resume     = adp5588_led_resume,
#endif
   .driver     = {
   .name	      = "adp5588-leds",
   .owner      = THIS_MODULE,
   },
};

static int __init 
adp5588_led_init(void)
{
	return platform_driver_register(&adp5588_led_driver);
}
module_init(adp5588_led_init);

static void __exit 
adp5588_led_exit(void)
{
	platform_driver_unregister(&adp5588_led_driver);
}
module_exit(adp5588_led_exit);



MODULE_DESCRIPTION("ADP5588 QWERTY Keyboard LEDs driver");
MODULE_AUTHOR("Vinh Vo <vinh@motorola.com>")
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adp5588-leds");
