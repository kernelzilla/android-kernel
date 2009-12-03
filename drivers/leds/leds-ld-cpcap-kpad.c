/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/leds.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

struct keypad_led_data {
	struct cpcap_device *cpcap;
	struct led_classdev ld_cpcap_keypad_class_dev;
};

static void ld_cpcap_kpad_store(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct keypad_led_data *keypad_data =
	    container_of(led_cdev, struct keypad_led_data,
			 ld_cpcap_keypad_class_dev);
	int cpcap_status = 0;

	if (value != LED_OFF) {
		cpcap_status = cpcap_regacc_write(keypad_data->cpcap,
			CPCAP_REG_GPIO6, CPCAP_BIT_GPIO6DRV,
			CPCAP_BIT_GPIO6DRV);
	} else {
		cpcap_status = cpcap_regacc_write(keypad_data->cpcap,
			CPCAP_REG_GPIO6, 0,
			CPCAP_BIT_GPIO6DRV);
	}
}
static int ld_cpcap_kpad_probe(struct platform_device *pdev)
{
	int ret;
	struct keypad_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;

	}
	info = kzalloc(sizeof(struct keypad_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	info->cpcap = pdev->dev.platform_data;
	platform_set_drvdata(pdev, info);

	ret = cpcap_regacc_write(info->cpcap,
			CPCAP_REG_GPIO6, CPCAP_BIT_GPIO6DIR,
			CPCAP_BIT_GPIO6DIR);
	if (ret < 0) {
		pr_err("%s: Writing CPCAP failed: \n", __func__);
		kfree(info);
		return ret;
	}

	info->ld_cpcap_keypad_class_dev.name = LD_KPAD_DEV;
	info->ld_cpcap_keypad_class_dev.brightness_set = ld_cpcap_kpad_store;

	ret = led_classdev_register(&pdev->dev,
				    &info->ld_cpcap_keypad_class_dev);
	if (ret < 0) {
		pr_err("%s: Register led class failed: \n", __func__);
		kfree(info);
		return ret;
	}

	return ret;
}

static int ld_cpcap_kpad_remove(struct platform_device *pdev)
{
	struct keypad_led_data *info = platform_get_drvdata(pdev);

	led_classdev_unregister(&info->ld_cpcap_keypad_class_dev);
	return 0;
}

static struct platform_driver ld_cpcap_kpad_driver = {
	.probe = ld_cpcap_kpad_probe,
	.remove = ld_cpcap_kpad_remove,
	.driver = {
		   .name = LD_KPAD_DEV,
		   .owner = THIS_MODULE,
		   },
};

static int __init ld_cpcap_kpad_init(void)
{
	return platform_driver_register(&ld_cpcap_kpad_driver);
}

static void __exit ld_cpcap_kpad_exit(void)
{
	platform_driver_unregister(&ld_cpcap_kpad_driver);
}

module_init(ld_cpcap_kpad_init);
module_exit(ld_cpcap_kpad_exit);

MODULE_DESCRIPTION("CPCAP Keypad Lighting driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
