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

#include <linux/err.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/leds-cpcap-kpad.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

struct kpad_led {
	struct cpcap_device *cpcap;
	struct led_classdev cpcap_kpad_dev;
	struct regulator *regulator;
	int regulator_state;
};

static void cpcap_kpad_led_set(struct led_classdev *led_cdev,
			  enum led_brightness value)
{
	int cpcap_status = 0;
	struct cpcap_platform_data *data;
	struct spi_device *spi;
	struct cpcap_leds *leds;

	struct kpad_led *kpad_led =
	    container_of(led_cdev, struct kpad_led,
			 cpcap_kpad_dev);

	spi = kpad_led->cpcap->spi;
	data = (struct cpcap_platform_data *)spi->controller_data;
	leds = data->leds;

	if (value > LED_OFF) {
		if ((kpad_led->regulator) &&
		    (kpad_led->regulator_state == 0)) {
			regulator_enable(kpad_led->regulator);
			kpad_led->regulator_state = 1;
		}

		cpcap_status = cpcap_regacc_write(kpad_led->cpcap,
						  leds->kpad_led.kpad_reg,
						  leds->kpad_led.kpad_on,
						  leds->kpad_led.kpad_mask);
	} else {
		if ((kpad_led->regulator) &&
		    (kpad_led->regulator_state == 1)) {
			regulator_disable(kpad_led->regulator);
			kpad_led->regulator_state = 0;
		}
		cpcap_status = cpcap_regacc_write(kpad_led->cpcap,
						  leds->kpad_led.kpad_reg,
						  leds->kpad_led.kpad_off,
						  leds->kpad_led.kpad_mask);
	}
	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	return;
}

static int cpcap_kpad_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct kpad_led *led;

	if (pdev == NULL) {
		pr_err("%s: platform data required %d\n", __func__, -ENODEV);
		return -ENODEV;

	}
	led = kzalloc(sizeof(struct kpad_led), GFP_KERNEL);
	if (led == NULL) {
		pr_err("%s: Unable to allocate memory %d\n", __func__, -ENOMEM);
		ret = -ENOMEM;
	}

	led->cpcap = pdev->dev.platform_data;
	platform_set_drvdata(pdev, led);

	led->regulator = regulator_get(&pdev->dev, "sw5");
	if (IS_ERR(led->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__, "sw5");
		ret = PTR_ERR(led->regulator);
		goto exit_request_reg_failed;
	}

	led->regulator_state = 0;

	led->cpcap_kpad_dev.name = CPCAP_KPAD_DEV;
	led->cpcap_kpad_dev.brightness_set = cpcap_kpad_led_set;
	ret = led_classdev_register(&pdev->dev, &led->cpcap_kpad_dev);
	if (ret) {
		printk(KERN_ERR "Register kpad led class failed %d\n", ret);
		goto err_reg_kpad_failed;
	}

	return ret;

err_reg_kpad_failed:
	if (led->regulator)
		regulator_put(led->regulator);

exit_request_reg_failed:
	kfree(led);
	return ret;
}

static int cpcap_kpad_led_remove(struct platform_device *pdev)
{
	struct kpad_led *led = platform_get_drvdata(pdev);

	if (led->regulator)
		regulator_put(led->regulator);

	led_classdev_unregister(&led->cpcap_kpad_dev);
	kfree(led);
	return 0;
}

static struct platform_driver cpcap_kpad_driver = {
	.probe   = cpcap_kpad_led_probe,
	.remove  = cpcap_kpad_led_remove,
	.driver  = {
		.name  = CPCAP_KPAD_DEV,
	},
};

#ifdef CONFIG_ARM_OF
static int lights_of_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_KPAD_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_KPAD_LED);
		return -ENODEV;
	}

	prop = of_get_property(node, "ruth_kpad_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_RUTH_KPAD_LED);
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}
#endif

static int cpcap_kpad_led_init(void)
{
#ifdef CONFIG_ARM_OF
	int err = lights_of_init();
	if (err <= 0) {
		pr_err("Ruth keyboard led device declared unavailable: %d\n",
			err);
		return err;
	}
#endif
	return platform_driver_register(&cpcap_kpad_driver);
}

static void cpcap_kpad_led_shutdown(void)
{
	platform_driver_unregister(&cpcap_kpad_driver);
}

module_init(cpcap_kpad_led_init);
module_exit(cpcap_kpad_led_shutdown);

MODULE_DESCRIPTION("Icon/Kpad Lighting Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GNU");






