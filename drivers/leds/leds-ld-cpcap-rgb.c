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

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

struct msg_ind_led_data {
	struct led_classdev msg_ind_red_class_dev;
	struct led_classdev msg_ind_green_class_dev;
	struct led_classdev msg_ind_blue_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

void msg_ind_set_rgb_brightness(struct msg_ind_led_data *msg_ind_data,
				int color, enum led_brightness value)
{
#ifdef CONFIG_LEDS_SHOLEST
	unsigned short brightness = LD_MSG_IND_LO_CURRENT | LD_MSG_IND_ON;
#else
	unsigned short brightness = LD_MSG_IND_CURRENT | LD_MSG_IND_ON;
#endif
	int cpcap_status = 0;
	int cpcap_register = 0;

	if (color & LD_LED_RED)
#ifdef CONFIG_LEDS_SHOLEST
        cpcap_register = CPCAP_REG_ADLC;
#else
		cpcap_register = CPCAP_REG_REDC;
#endif
	else if (color & LD_LED_GREEN)
		cpcap_register = CPCAP_REG_GREENC;
	else if (color & LD_LED_BLUE)
		cpcap_register = CPCAP_REG_BLUEC;

	if (value == LED_OFF) {
		/* Due to a HW issue turn off the current then
		turn off the duty cycle */
		brightness = 0x01;
		cpcap_status = cpcap_regacc_write(msg_ind_data->cpcap,
					  cpcap_register, brightness,
					  LD_MSG_IND_CPCAP_MASK);

		brightness = 0x00;
	}
#ifdef CONFIG_LEDS_SHOLEST
	else if (value <= 51)
		brightness |= (LD_MSG_IND_LOW << \
				((cpcap_register == CPCAP_REG_ADLC) << 1));
	else if (value <= 104)
		brightness |= (LD_MSG_IND_LOW_MED << \
				((cpcap_register == CPCAP_REG_ADLC) << 1));
	else if (value <= 155)
		brightness |= (LD_MSG_IND_MEDIUM << \
				((cpcap_register == CPCAP_REG_ADLC) << 1));
	else if (value <= 201)
		brightness |= (LD_MSG_IND_MED_HIGH << \
				((cpcap_register == CPCAP_REG_ADLC) << 1));
	else
		brightness |= (LD_MSG_IND_HIGH << \
				((cpcap_register == CPCAP_REG_ADLC) << 1));
#else
    else if (value <= 51)
		brightness |= LD_MSG_IND_LOW;
	else if (value <= 104)
		brightness |= LD_MSG_IND_LOW_MED;
	else if (value <= 155)
		brightness |= LD_MSG_IND_MEDIUM;
	else if (value <= 201)
		brightness |= LD_MSG_IND_MED_HIGH;
	else
		brightness |= LD_MSG_IND_HIGH;
#endif

	cpcap_status = cpcap_regacc_write(msg_ind_data->cpcap,
					  cpcap_register, brightness,
					  LD_MSG_IND_CPCAP_MASK);

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	if (value > LED_OFF) {
		if (!(msg_ind_data->regulator_state & color)) {
			if (msg_ind_data->regulator) {
				regulator_enable(msg_ind_data->regulator);
				msg_ind_data->regulator_state |= color;
			}
		}
	} else {
		if (msg_ind_data->regulator_state & color) {
			if (msg_ind_data->regulator) {
				regulator_disable(msg_ind_data->regulator);
				msg_ind_data->regulator_state &= ~color;
			}
		}
	}

	return;
}

static void msg_ind_red_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct msg_ind_led_data *msg_ind_data =
	    container_of(led_cdev, struct msg_ind_led_data,
			 msg_ind_red_class_dev);

	msg_ind_set_rgb_brightness(msg_ind_data, LD_LED_RED, value);
}

static void msg_ind_green_set(struct led_classdev *led_cdev,
			      enum led_brightness value)
{
	struct msg_ind_led_data *msg_ind_data =
	    container_of(led_cdev, struct msg_ind_led_data,
			 msg_ind_green_class_dev);

	msg_ind_set_rgb_brightness(msg_ind_data, LD_LED_GREEN, value);
}

static void msg_ind_blue_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct msg_ind_led_data *msg_ind_data =
	    container_of(led_cdev, struct msg_ind_led_data,
			 msg_ind_blue_class_dev);

	msg_ind_set_rgb_brightness(msg_ind_data, LD_LED_BLUE, value);
}

static ssize_t
msg_ind_blink(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct msg_ind_led_data *msg_ind_data = dev_get_drvdata(dev);
	unsigned long led_blink = LED_OFF;
	int ret;

	ret = strict_strtoul(buf, 10, &led_blink);
	if (ret != 0) {
		pr_err("%s: Invalid parameter sent\n", __func__);
		return -1;
	}
	if (led_blink > LED_OFF)
		cpcap_uc_start(msg_ind_data->cpcap, CPCAP_MACRO_6);
	else
		cpcap_uc_stop(msg_ind_data->cpcap, CPCAP_MACRO_6);

	return 0;
}

static DEVICE_ATTR(blink, 0644, NULL, msg_ind_blink);

static int msg_ind_rgb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct msg_ind_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;

	}
	info = kzalloc(sizeof(struct msg_ind_led_data), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	info->cpcap = pdev->dev.platform_data;
	platform_set_drvdata(pdev, info);

	info->regulator = regulator_get(&pdev->dev, LD_SUPPLY);
	if (IS_ERR(info->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__, LD_SUPPLY);
		ret = PTR_ERR(info->regulator);
		goto exit_request_reg_failed;
	}

	info->regulator_state = 0;
	info->msg_ind_red_class_dev.name = "red";
	info->msg_ind_red_class_dev.brightness_set = msg_ind_red_set;
	ret = led_classdev_register(&pdev->dev, &info->msg_ind_red_class_dev);
	if (ret < 0) {
		pr_err("%s:Register Red LED class failed\n", __func__);
		goto err_reg_red_class_failed;
	}

	ret = device_create_file(info->msg_ind_red_class_dev.dev,
				 &dev_attr_blink);
	if (ret < 0) {
		pr_err("%s: File device creation failed: %d\n", __func__, ret);
		goto err_create_blink_failed;
	}

	info->msg_ind_green_class_dev.name = "green";
	info->msg_ind_green_class_dev.brightness_set = msg_ind_green_set;
	ret = led_classdev_register(&pdev->dev, &info->msg_ind_green_class_dev);
	if (ret < 0) {
		pr_err("%s: Register Green LED class failed\n", __func__);
		goto err_reg_green_class_failed;
	}

	info->msg_ind_blue_class_dev.name = "blue";
	info->msg_ind_blue_class_dev.brightness_set = msg_ind_blue_set;
	ret = led_classdev_register(&pdev->dev, &info->msg_ind_blue_class_dev);
	if (ret < 0) {
		pr_err("%s: Register blue LED class failed\n", __func__);
		goto err_reg_blue_class_failed;
	}

	return ret;

err_reg_blue_class_failed:
	led_classdev_unregister(&info->msg_ind_green_class_dev);
err_reg_green_class_failed:
	device_remove_file(info->msg_ind_red_class_dev.dev, &dev_attr_blink);
err_create_blink_failed:
	led_classdev_unregister(&info->msg_ind_red_class_dev);
err_reg_red_class_failed:
	if (info->regulator)
		regulator_put(info->regulator);
exit_request_reg_failed:
	kfree(info);
	return ret;
}

static int msg_ind_rgb_remove(struct platform_device *pdev)
{
	struct msg_ind_led_data *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);

	device_remove_file(info->msg_ind_red_class_dev.dev, &dev_attr_blink);

	led_classdev_unregister(&info->msg_ind_red_class_dev);
	led_classdev_unregister(&info->msg_ind_green_class_dev);
	led_classdev_unregister(&info->msg_ind_blue_class_dev);
	return 0;
}

static struct platform_driver ld_msg_ind_rgb_driver = {
	.probe = msg_ind_rgb_probe,
	.remove = msg_ind_rgb_remove,
	.driver = {
		   .name = LD_MSG_IND_DEV,
		   },
};

static int lights_of_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_NOTIFICATION_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_NOTIFICATION_LED);
		return -ENODEV;
	}

	prop = of_get_property(node, "tablet_rgb_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_TABLET_RGB_LED);
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int __init ld_msg_ind_rgb_init(void)
{
#ifdef CONFIG_ARM_OF
	int err = lights_of_init();
	if (err <= 0) {
		pr_err("Tablet RGB led device declared unavailable: %d\n", err);
		return err;
	}
#endif
	return platform_driver_register(&ld_msg_ind_rgb_driver);
}

static void __exit ld_msg_ind_rgb_exit(void)
{
	platform_driver_unregister(&ld_msg_ind_rgb_driver);
}

module_init(ld_msg_ind_rgb_init);
module_exit(ld_msg_ind_rgb_exit);

MODULE_DESCRIPTION("Message Indicator Lighting driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
