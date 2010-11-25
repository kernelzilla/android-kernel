/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <mach/pmic.h>

#define MAX_KEYPAD_BL_LEVEL	16

/* IKPITTSBURGH - 138 */
#ifdef CONFIG_MACH_PITTSBURGH
#ifndef FALSE
#define FALSE  0
#define TRUE   (!FALSE)
#endif
#define KEYPAD_BL_TEST_ON_OFF  0x0B    /* 'q' keypad map value which toggle ON/OFF the keypad bl */
static bool pittsburgh_keypad_backlight_test = FALSE;
static int  keypad_bl_value = 0;

static void msm_keypad_bl_led_value_adjusted (enum led_brightness *value)
{
   if (*value == KEYPAD_BL_TEST_ON_OFF)
   {
      /* Toggle ON/OFF the keypad backlight test */
      pittsburgh_keypad_backlight_test = 
                        (pittsburgh_keypad_backlight_test == FALSE? TRUE: FALSE);

   }
   else
   {
      keypad_bl_value = *value;   /* save the current value */
   }

   if (pittsburgh_keypad_backlight_test == TRUE)
   {
      /* keypad bl decense test is ON */
      *value = LED_FULL;   /* Always turn ON when in test mode */
   } 
   else
   {
      /* keypad bl decense test is OFF */
      *value = keypad_bl_value;   /* retrieve the previous value */
   }
}
 
#endif/* CONFIG_MACH_PITTSBURGH */

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;

#ifdef CONFIG_MACH_PITTSBURGH
        msm_keypad_bl_led_value_adjusted (&value);
#endif/* CONFIG_MACH_PITTSBURGH */

	ret = pmic_set_led_intensity(LED_KEYPAD, value / MAX_KEYPAD_BL_LEVEL);
	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct led_classdev msm_kp_bl_led = {
	.name			= "keyboard-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	return rc;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
