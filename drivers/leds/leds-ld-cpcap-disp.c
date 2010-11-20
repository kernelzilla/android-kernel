/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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

struct disp_button_led_data {
	struct led_classdev disp_button_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

struct disp_button_config_data {
	u16 duty_cycle;
	u16 cpcap_mask;
	u16 abmode_cpcap_mask;
	u16 pwm;
	u16 fade_time;
	u16 fade_en;
	u16 abmode;
	u8 led_current;
	enum cpcap_reg reg;
	enum cpcap_reg reg2;
};
static struct disp_button_config_data btn_data;

static void disp_button_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	unsigned short brightness = 0;
	int cpcap_status = 0;
	struct disp_button_config_data *pbtn_data = &btn_data;
	struct disp_button_led_data *disp_button_led_data =
	    container_of(led_cdev, struct disp_button_led_data,
			 disp_button_class_dev);

	if (value > 0) {

		brightness = (pbtn_data->duty_cycle |
				pbtn_data->led_current | LD_DISP_BUTTON_ON);

		if ((disp_button_led_data->regulator) &&
		    (disp_button_led_data->regulator_state == 0)) {
			regulator_enable(disp_button_led_data->regulator);
			disp_button_led_data->regulator_state = 1;
		}

		if (pbtn_data->reg2) {
			brightness |= (pbtn_data->pwm | pbtn_data->fade_time |
					pbtn_data->fade_en);
		}

		cpcap_status = cpcap_regacc_write(disp_button_led_data->cpcap,
						  pbtn_data->reg,
						  brightness,
						  pbtn_data->cpcap_mask);
		if (cpcap_status < 0)
			pr_err("%s: Writing to the register failed for %i\n",
			       __func__, cpcap_status);

		if (pbtn_data->reg2) {
			cpcap_status = cpcap_regacc_write(
						disp_button_led_data->cpcap,
						pbtn_data->reg2,
						pbtn_data->abmode,
						pbtn_data->abmode_cpcap_mask);
			if (cpcap_status < 0)
				pr_err(
					"%s: Writing to the register failed\
					for %i\n", __func__, cpcap_status);
		}
	} else {
		if ((disp_button_led_data->regulator) &&
		    (disp_button_led_data->regulator_state == 1)) {
			regulator_disable(disp_button_led_data->regulator);
			disp_button_led_data->regulator_state = 0;
		}
		/* Due to a HW issue turn off the current then
		turn off the duty cycle */
		brightness = 0x01;
		cpcap_status = cpcap_regacc_write(disp_button_led_data->cpcap,
						  pbtn_data->reg,
						  brightness,
						  pbtn_data->cpcap_mask);

		brightness = 0x00;
		cpcap_status = cpcap_regacc_write(disp_button_led_data->cpcap,
						  pbtn_data->reg,
						  brightness,
						  pbtn_data->cpcap_mask);
		if (cpcap_status < 0)
			pr_err("%s: Writing to the register failed for %i\n",
			       __func__, cpcap_status);
	}

}
EXPORT_SYMBOL(disp_button_set);

static int disp_button_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct disp_button_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;

	}
	info = kzalloc(sizeof(struct disp_button_led_data), GFP_KERNEL);
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

	info->disp_button_class_dev.name = "button-backlight";
	info->disp_button_class_dev.brightness_set = disp_button_set;
	ret = led_classdev_register(&pdev->dev, &info->disp_button_class_dev);
	if (ret < 0) {
		pr_err("%s:Register button backlight class failed\n", __func__);
		goto err_reg_button_class_failed;
	}

	return ret;

err_reg_button_class_failed:
	if (info->regulator)
		regulator_put(info->regulator);
exit_request_reg_failed:
	kfree(info);
	return ret;
}

static int disp_button_remove(struct platform_device *pdev)
{
	struct disp_button_led_data *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);

	led_classdev_unregister(&info->disp_button_class_dev);
	return 0;
}

static struct platform_driver ld_disp_button_driver = {
	.probe = disp_button_probe,
	.remove = disp_button_remove,
	.driver = {
		   .name = LD_DISP_BUTTON_DEV,
		   },
};

#ifdef CONFIG_ARM_OF
static int lights_of_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;
	struct disp_button_config_data *pbtn_data = &btn_data;

	node = of_find_node_by_path(DT_HOME_LED);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_HOME_LED);
		return -ENODEV;
	}

	prop = of_get_property(node, "tablet_button_led", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
	    pr_err("Read property %s error!\n", DT_PROP_TABLET_BUTTON);
		of_node_put(node);
		return -ENODEV;
	}

    /* BEGIN Motorola, hvjk38, 12/15/09, IKMAP-2703
    Hard Keys do not light making it difficult to see in low/no light */
    if (device_available > 0) {
    /* END IKMAP-2703 */
        prop = of_get_property(node, "button_duty_cycle", NULL);
        if (prop)
            pbtn_data->duty_cycle = *(u16 *)prop;
        else {
            pr_err("Read property %s error!\n", "button_duty_cycle");
            of_node_put(node);
            return -ENODEV;
        }

       prop = of_get_property(node, "button_cpcap_mask", NULL);
       if (prop)
           pbtn_data->cpcap_mask = *(u16 *)prop;
       else {
           pr_err("Read property %s error!\n", "button_cpcap_mask");
           of_node_put(node);
           return -ENODEV;
       }

       prop = of_get_property(node, "button_current", NULL);
       if (prop)
           pbtn_data->led_current = *(u8 *)prop;
       else {
           pr_err("Read property %s error!\n", "button_current");
           of_node_put(node);
           return -ENODEV;
       }

       prop = of_get_property(node, "button_register", NULL);
       if (prop)
           pbtn_data->reg = *(enum cpcap_reg *)prop;
       else {
           pr_err("Read property %s error!\n", "button_register");
           of_node_put(node);
           return -ENODEV;
       }

	prop = of_get_property(node, "button_register2", NULL);
	if (prop) {
		pbtn_data->reg2 = *(enum cpcap_reg *)prop;
		prop = of_get_property(node, "button_cpcap_abmode_mask", NULL);
		if (prop)
			pbtn_data->abmode_cpcap_mask = *(u16 *)prop;
		else {
			pr_err("Read property %s error!\n",
				"button_cpcap_abmode_mask");
			of_node_put(node);
			return -ENODEV;
		}

		prop = of_get_property(node, "button_pwm", NULL);
		if (prop)
			pbtn_data->pwm = *(u16 *)prop;
		else {
			pr_err("Read property %s error!\n", "button_pwm");
			of_node_put(node);
			return -ENODEV;
		}

		prop = of_get_property(node, "button_fade_time", NULL);
		if (prop)
			pbtn_data->fade_time = *(u16 *)prop;
		else {
			pr_err("Read property %s error!\n", "button_fade_time");
			of_node_put(node);
			return -ENODEV;
		}

		prop = of_get_property(node, "button_fade_en", NULL);
		if (prop)
			pbtn_data->fade_en = *(u16 *)prop;
		else {
			pr_err("Read property %s error!\n", "button_fade_time");
			of_node_put(node);
			return -ENODEV;
		}

		prop = of_get_property(node, "button_abmode", NULL);
		if (prop)
			pbtn_data->abmode = *(u16 *)prop;
		else {
			pr_err("Read property %s error!\n", "button_abmode");
			of_node_put(node);
			return -ENODEV;
		}
	} else {
		pr_err("Read property %s error!\n", "button_register2");
		pbtn_data->reg2 = 0;
	}
    }
    /* BEGIN Motorola, hvjk38, 12/15/09, IKMAP-2703
    Hard Keys do not light making it difficult to see in low/no light */
    else
        pr_err("Tablet button led device not present: %d\n", device_available);
    /* END IKMAP-2703 */

	of_node_put(node);
	return device_available;
}
#endif

static int __init led_disp_button_init(void)
{
#ifdef CONFIG_ARM_OF
	int err = lights_of_init();
	if (err <= 0) {
		pr_err("Tablet button led device declared unavailable: %d\n",
			err);
		return err;
	}
#endif
	return platform_driver_register(&ld_disp_button_driver);
}

static void __exit led_disp_button_exit(void)
{
	platform_driver_unregister(&ld_disp_button_driver);
}

module_init(led_disp_button_init);
module_exit(led_disp_button_exit);

MODULE_DESCRIPTION("Display Button Lighting driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
