/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free dispware; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free dispware Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free dispware
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/leds.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

struct af_led_data {
	struct led_classdev af_led_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

static void af_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	unsigned short brightness = 0;
	int cpcap_status = 0;
	struct af_led_data *af_led_data =
	    container_of(led_cdev, struct af_led_data,
			 af_led_class_dev);

	if (value > 0) {
		/* need to add dynamic control */
		brightness = LD_MSG_IND_CPCAP_MASK >> 2;

		if ((af_led_data->regulator) &&
		    (af_led_data->regulator_state == 0)) {
			regulator_enable(af_led_data->regulator);
			af_led_data->regulator_state = 1;
		}

		cpcap_status = cpcap_regacc_write(af_led_data->cpcap,
			  CPCAP_REG_REDC, brightness, LD_MSG_IND_CPCAP_MASK);
		if (cpcap_status < 0)
			pr_err("%s: Writing to the register failed for %i\n",
			       __func__, cpcap_status);

	} else {
		brightness = 0;

		if ((af_led_data->regulator) &&
		    (af_led_data->regulator_state == 1)) {
			regulator_disable(af_led_data->regulator);
			af_led_data->regulator_state = 0;
		}

		cpcap_status = cpcap_regacc_write(af_led_data->cpcap,
			  CPCAP_REG_REDC, brightness, LD_MSG_IND_CPCAP_MASK);
		if (cpcap_status < 0)
			pr_err("%s: Writing to the register failed for %i\n",
			       __func__, cpcap_status);
	}

}
EXPORT_SYMBOL(af_led_set);

static int af_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct af_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct af_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	info->cpcap = pdev->dev.platform_data;
	platform_set_drvdata(pdev, info);

	info->regulator = regulator_get(&pdev->dev, LD_SUPPLY);
	if (IS_ERR(info->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__, LD_SUPPLY);
		ret = PTR_ERR(info->regulator);
		goto exit_request_reg_failed;

	}

	info->regulator_state = 0;

	info->af_led_class_dev.name = "af-led";
	info->af_led_class_dev.brightness_set = af_led_set;
	ret = led_classdev_register(&pdev->dev, &info->af_led_class_dev);
	if (ret < 0) {
		pr_err("%s:Register Auto focus LED class failed\n", __func__);
		goto err_reg_af_led_class_failed;
	}

	return ret;

err_reg_af_led_class_failed:
	if (info->regulator)
		regulator_put(info->regulator);
exit_request_reg_failed:
	kfree(info);
	return ret;
}

static int af_remove(struct platform_device *pdev)
{
	struct af_led_data *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);

	led_classdev_unregister(&info->af_led_class_dev);
	return 0;
}

static struct platform_driver ld_af_driver = {
	.probe = af_probe,
	.remove = af_remove,
	.driver = {
		   .name = LD_AF_LED_DEV,
		   },
};

static int __init led_af_init(void)
{
	return platform_driver_register(&ld_af_driver);
}

static void __exit led_af_exit(void)
{
	platform_driver_unregister(&ld_af_driver);
}

module_init(led_af_init);
module_exit(led_af_exit);

MODULE_DESCRIPTION("AF LED driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
