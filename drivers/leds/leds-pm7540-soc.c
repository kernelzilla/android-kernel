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
 * PM7540 Charging LED Driver for Sesame
 *
 * Jiaping Lv a19234@motorola.com
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/pmic.h>



static void pm7540_soc_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;

       printk (KERN_ERR "input charging led is %d\n", value);
	ret = pmic_set_led_intensity( LED_LCD, value ? 2 : 0);
	if (ret)
		dev_err(led_cdev->dev, "can't set charging backlight\n");
}

static struct led_classdev pm7540_soc_led = {
	.name			= "charging",
	.brightness_set		= pm7540_soc_led_set,
	.brightness		= LED_OFF,
};

static int pm7540_soc_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &pm7540_soc_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	pm7540_soc_led_set(&pm7540_soc_led, LED_OFF);
	return rc;
}

static int __devexit pm7540_soc_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&pm7540_soc_led);

	return 0;
}

#ifdef CONFIG_PM
static int pm7540_soc_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&pm7540_soc_led);

	return 0;
}

static int pm7540_soc_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&pm7540_soc_led);

	return 0;
}
#else
#define pm7540_soc_led_suspend NULL
#define pm7540_soc_led_resume NULL
#endif

static struct platform_driver pm7540_soc_led_driver = {
	.probe		= pm7540_soc_led_probe,
	.remove		= __devexit_p(pm7540_soc_led_remove),
	.suspend	= pm7540_soc_led_suspend,
	.resume		= pm7540_soc_led_resume,
	.driver		= {
		.name	= "pm7540-soc-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init pm7540_soc_led_init(void)
{
	return platform_driver_register(&pm7540_soc_led_driver);
}
module_init(pm7540_soc_led_init);

static void __exit pm7540_soc_led_exit(void)
{
	platform_driver_unregister(&pm7540_soc_led_driver);
}
module_exit(pm7540_soc_led_exit);

MODULE_DESCRIPTION("MSM Sign of Charging LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:soc-leds");
