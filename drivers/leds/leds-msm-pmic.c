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

#ifdef CONFIG_MACH_SOCIAL
#define MAX_USB_INDICATOR_LEVEL	16
#define FLASH_LED_CURRENT_VALUE 160
#endif /* CONFIG_MACH_SOCIAL */

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;

	ret = pmic_set_led_intensity( LED_KEYPAD, value ? 2 : 0);
	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

#ifdef CONFIG_MACH_SOCIAL
static void msm_usb_led_set(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	int ret;

	ret = pmic_set_led_intensity(LED_LCD, value / MAX_USB_INDICATOR_LEVEL );
	if (ret)
		dev_err(led_cdev->dev, "can't set usb indicator\n");
}

static void msm_button_bl_led_set(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	int ret;
	int cu_value = 0;

	ret = pmic_flash_led_set_mode(FLASH_LED_MODE__MANUAL);
			
	if (value != 0)
	   cu_value = FLASH_LED_CURRENT_VALUE;
        ret = pmic_flash_led_set_current(cu_value);
  	if (ret)
		dev_err(led_cdev->dev, "can't set button  key backlight\n");
}
#endif /* CONFIG_MACH_SOCIAL */

static struct led_classdev msm_kp_bl_led = {
	.name			= "keyboard-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

#ifdef CONFIG_MACH_SOCIAL
static struct led_classdev msm_usb_led = {
	.name			= "usb-indicator",
	.brightness_set		= msm_usb_led_set,
	.brightness		= LED_OFF,
};

static struct led_classdev msm_button_bl_led = {
	.name			= "button-backlight",
	.brightness_set		= msm_button_bl_led_set,
	.brightness		= LED_OFF,
};
#endif /* CONFIG_MACH_SOCIAL */

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	
#ifdef CONFIG_MACH_SOCIAL
	rc = led_classdev_register(&pdev->dev, &msm_usb_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register usb indicator driver\n");
		return rc;
	}
	rc = led_classdev_register(&pdev->dev, &msm_button_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register button backlight driver\n");
		return rc;
	}
#endif /* CONFIG_MACH_SOCIAL */

	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	
#ifdef CONFIG_MACH_SOCIAL
	msm_keypad_bl_led_set(&msm_usb_led, LED_OFF);
	msm_keypad_bl_led_set(&msm_button_bl_led, LED_OFF);
#endif /* CONFIG_MACH_SOCIAL */

	return rc;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);
#ifdef CONFIG_MACH_SOCIAL
	led_classdev_unregister(&msm_usb_led);
	led_classdev_unregister(&msm_button_bl_led);
#endif /* CONFIG_MACH_SOCIAL */

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
