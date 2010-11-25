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
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/leds-cpcap-display.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/earlysuspend.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

struct display_led {
	struct input_dev *idev;
	struct platform_device *pdev;
	struct cpcap_device *cpcap;
	struct led_classdev cpcap_display_dev;
	struct delayed_work dwork;
	struct work_struct work;
	struct early_suspend suspend;
	struct cpcap_leds *leds;
	struct regulator *regulator;
	int regulator_state;
	int prev_als_reading;
	int light_percent;
	u8 mode;
	u8 curr_zone;
	u8 prev_zone;
	u8 last_brightness;
	bool phone_awake;
};

static void cpcap_display_led_set(struct led_classdev *led_cdev,
			  enum led_brightness value)
{
	int cpcap_status = 0;
	u16 cpcap_backlight_set = 0;
	struct cpcap_leds *leds;

	struct display_led *display_led =
	    container_of(led_cdev, struct display_led,
			 cpcap_display_dev);

	leds = display_led->leds;

	if (value > LED_OFF) {
		if ((display_led->regulator) &&
		    (display_led->regulator_state == 0)) {
			regulator_enable(display_led->regulator);
			display_led->regulator_state = 1;
		}

		/*
		 * set value to PWM bits 5 - 11
		 */
		display_led->phone_awake = true;
		display_led->last_brightness = value;

		if (display_led->mode == AUTOMATIC)
			value = (value * display_led->light_percent) / 100;

		cpcap_backlight_set = (value / 2) << 5;
		cpcap_backlight_set |= leds->display_led.display_init;
		cpcap_status = cpcap_regacc_write(display_led->cpcap,
						  leds->display_led.display_reg,
						  cpcap_backlight_set,
						  leds->
						    display_led.display_mask);
	} else {
		if ((display_led->regulator) &&
		    (display_led->regulator_state)) {
			regulator_disable(display_led->regulator);
			display_led->regulator_state = 0;
		}

		display_led->phone_awake = false;
		cpcap_status = cpcap_regacc_write(display_led->cpcap,
						  leds->display_led.display_reg,
						  leds->display_led.display_off,
						  leds->
						    display_led.display_mask);
	}
	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);
	return;
}

static void cpcap_display_led_read_a2d(struct display_led *d_led)
{
	int err;
	u16 brightness;
	struct cpcap_adc_request request;
	struct cpcap_device *cpcap = d_led->cpcap;

	request.format = CPCAP_ADC_FORMAT_RAW;
	request.timing = CPCAP_ADC_TIMING_IMM;
	request.type = CPCAP_ADC_TYPE_BANK_1;

	err = cpcap_adc_sync_read(cpcap, &request);
	if (err < 0) {
		pr_err("%s: A2D read error %d\n", __func__, err);
		return;
	}

	if (request.result[CPCAP_ADC_TSY1_AD14] <= 150) {
		d_led->curr_zone = 0;
		d_led->light_percent = d_led->leds->display_led.zone0;
	} else if (request.result[CPCAP_ADC_TSY1_AD14] <= 300) {
		d_led->curr_zone = 1;
		d_led->light_percent = d_led->leds->display_led.zone1;
	} else if (request.result[CPCAP_ADC_TSY1_AD14] <= 450) {
		d_led->curr_zone = 2;
		d_led->light_percent = d_led->leds->display_led.zone2;
	} else if (request.result[CPCAP_ADC_TSY1_AD14] <= 600) {
		d_led->curr_zone = 3;
		d_led->light_percent = d_led->leds->display_led.zone3;
	} else if (request.result[CPCAP_ADC_TSY1_AD14] <= 750) {
		d_led->curr_zone = 4;
		d_led->light_percent = d_led->leds->display_led.zone4;
	} else {
		pr_err("Invalid light sensor input, ignore ...\n");
		return;
	}

	if (d_led->curr_zone != d_led->prev_zone) {
		brightness = (d_led->last_brightness *
			      d_led->light_percent) / 100;

		brightness = (brightness / 2) << 5;
		brightness |= d_led->leds->display_led.display_init;

		if (!d_led->phone_awake)
			goto do_not_set_light;

		err = cpcap_regacc_write(d_led->cpcap,
					 d_led->leds->display_led.display_reg,
					 brightness,
					 d_led->leds->display_led.display_mask);
		if (err < 0)
			pr_err("%s: Writing to the register failed for %i\n",
			       __func__, err);

		/*
		   TO DO
		   input_event(d_led->idev, EV_MSC, MSC_RAW, light_value);
		   input_sync(d_led->idev);
		*/
	}
do_not_set_light:
	d_led->prev_zone = d_led->curr_zone;
}

static void cpcap_display_led_work(struct work_struct *work)
{
	struct display_led *d_led =
	  container_of(work, struct display_led, work);

	cpcap_display_led_read_a2d(d_led);
}

static void cpcap_display_led_dwork(struct work_struct *work)
{
	struct display_led *d_led =
	  container_of((struct delayed_work *)work, struct display_led, dwork);

	cpcap_display_led_read_a2d(d_led);

	schedule_delayed_work(&d_led->dwork,
			      msecs_to_jiffies(d_led->leds->display_led.
					       poll_intvl));
}

static int cpcap_display_led_suspend(struct platform_device *pdev,
				     pm_message_t state)
{
	struct display_led *d_led = platform_get_drvdata(pdev);

	cancel_delayed_work(&d_led->dwork);
	cancel_delayed_work_sync(&d_led->dwork);

	if (d_led->regulator_state) {
		regulator_disable(d_led->regulator);
		d_led->regulator_state = 0;
	}
	return 0;
}

static int cpcap_display_led_resume(struct platform_device *pdev)
{
	struct display_led *d_led = platform_get_drvdata(pdev);

	schedule_delayed_work(&d_led->dwork,
			      msecs_to_jiffies(d_led->leds->display_led.
					       poll_intvl));

	if (0 == d_led->regulator_state) {
		regulator_enable(d_led->regulator);
		d_led->regulator_state = 1;
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cpcap_display_led_early_suspend(struct early_suspend *h)
{
	struct display_led *d_led =
	  container_of(h, struct display_led, suspend);

	cpcap_display_led_suspend(d_led->pdev, PMSG_SUSPEND);
}

static void cpcap_display_led_late_resume(struct early_suspend *h)
{
	struct display_led *d_led =
	  container_of(h, struct display_led, suspend);

	cpcap_display_led_resume(d_led->pdev);
}
#endif

static ssize_t cpcap_display_led_als_store(struct device *dev,
					   struct device_attribute
					   *attr, const char *buf, size_t size)
{
	int err;
	u16 brightness;
	unsigned long mode;
	struct display_led *d_led;
	struct platform_device *pdev =
	  container_of(dev->parent, struct platform_device, dev);

	d_led = platform_get_drvdata(pdev);

	if ((strict_strtoul(buf, 10, &mode)) < 0)
		return -1;

	if (mode >= AUTOMATIC) {
		d_led->mode = AUTOMATIC;
		schedule_delayed_work(&d_led->dwork,
				      msecs_to_jiffies(d_led->leds->display_led.
						       poll_intvl));
		brightness = (d_led->last_brightness *
			      d_led->light_percent) / 100;
	} else {
		d_led->mode = MANUAL;
		cancel_delayed_work(&d_led->dwork);
		cancel_delayed_work_sync(&d_led->dwork);
		brightness = d_led->last_brightness;
	}

	brightness = (brightness / 2) << 5;
	brightness |= d_led->leds->display_led.display_init;

	err = cpcap_regacc_write(d_led->cpcap,
				 d_led->leds->display_led.display_reg,
				 brightness,
				 d_led->leds->display_led.display_mask);
	if (err < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, err);

	return d_led->mode;
}

static DEVICE_ATTR(als, 0644, NULL, cpcap_display_led_als_store);

static int cpcap_display_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct display_led *d_led;

	struct cpcap_platform_data *data;
	struct spi_device *spi;

	if (pdev == NULL) {
		pr_err("%s: platform data required %d\n", __func__, -ENODEV);
		return -ENODEV;

	}
	d_led = kzalloc(sizeof(struct display_led), GFP_KERNEL);
	if (d_led == NULL) {
		pr_err("%s: Unable to allacate memory %d\n", __func__, -ENOMEM);
		return -ENOMEM;
	}

	d_led->pdev = pdev;
	d_led->cpcap = pdev->dev.platform_data;
	spi = d_led->cpcap->spi;
	data = (struct cpcap_platform_data *)spi->controller_data;
	d_led->leds = data->leds;
	d_led->mode = AUTOMATIC;
	platform_set_drvdata(pdev, d_led);

	d_led->idev = input_allocate_device();
	if (!d_led->idev) {
		ret = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
		       ret);
		goto err_input_allocate_failed;
	}

	d_led->idev->name = BACKLIGHT_ALS;
	input_set_capability(d_led->idev, EV_MSC, MSC_RAW);

	if (input_register_device(d_led->idev)) {
		pr_err("%s: input device register failed\n", __func__);
		goto err_input_register_failed;
	}

	d_led->regulator = regulator_get(&pdev->dev, "sw5");
	if (IS_ERR(d_led->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__, "sw5");
		ret = PTR_ERR(d_led->regulator);
		goto err_reg_request_failed;

	}
	d_led->regulator_state = 0;

	d_led->cpcap_display_dev.name = CPCAP_DISPLAY_DEV;
	d_led->cpcap_display_dev.brightness_set = cpcap_display_led_set;
	ret = led_classdev_register(&pdev->dev, &d_led->cpcap_display_dev);
	if (ret) {
		printk(KERN_ERR "Register display led class failed %d\n", ret);
		goto err_reg_led_class_failed;
	}

	if ((device_create_file(d_led->cpcap_display_dev.dev,
				&dev_attr_als)) < 0) {
		pr_err("%s:File device creation failed: %d\n",
		       __func__, -ENODEV);
		goto err_dev_create_failed;
	}

	INIT_WORK(&d_led->work, cpcap_display_led_work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	d_led->suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	d_led->suspend.suspend = cpcap_display_led_early_suspend;
	d_led->suspend.resume = cpcap_display_led_late_resume;
	register_early_suspend(&d_led->suspend);
#endif
	schedule_work(&d_led->work);

	INIT_DELAYED_WORK(&d_led->dwork, cpcap_display_led_dwork);
	schedule_delayed_work(&d_led->dwork,
			      msecs_to_jiffies(d_led->leds->display_led.
					       poll_intvl));
	return ret;

err_dev_create_failed:
	led_classdev_unregister(&d_led->cpcap_display_dev);
err_reg_led_class_failed:
	if (d_led->regulator)
		regulator_put(d_led->regulator);
err_reg_request_failed:
err_input_register_failed:
	input_free_device(d_led->idev);
err_input_allocate_failed:
	kfree(d_led);
	return ret;
}

static int cpcap_display_led_remove(struct platform_device *pdev)
{
	struct display_led *d_led = platform_get_drvdata(pdev);

	if (d_led->regulator)
		regulator_put(d_led->regulator);

	cancel_delayed_work(&d_led->dwork);
	cancel_delayed_work_sync(&d_led->dwork);

	device_remove_file(d_led->cpcap_display_dev.dev, &dev_attr_als);
	led_classdev_unregister(&d_led->cpcap_display_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&d_led->suspend);
#endif
	input_unregister_device(d_led->idev);
	kfree(d_led);
	return 0;
}

static struct platform_driver cpcap_display_led_driver = {
	.probe   = cpcap_display_led_probe,
	.remove  = cpcap_display_led_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = cpcap_display_led_suspend,
	.resume  = cpcap_display_led_resume,
#endif
	.driver  = {
		.name  = CPCAP_DISPLAY_DRV,
	},
};

static int lights_of_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_BACKLIGHT);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_BACKLIGHT);
		return -ENODEV;
	}

	prop = of_get_property(node, "ruth_lcd", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_RUTH_LCD);
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int cpcap_display_led_init(void)
{
#ifdef CONFIG_ARM_OF
	int err = lights_of_init();
	if (err <= 0) {
		pr_err("LCD Backlight device declared unavailable: %d\n", err);
		return err;
	}
#endif
	return platform_driver_register(&cpcap_display_led_driver);
}

static void cpcap_display_led_shutdown(void)
{
	platform_driver_unregister(&cpcap_display_led_driver);
}

module_init(cpcap_display_led_init);
module_exit(cpcap_display_led_shutdown);

MODULE_DESCRIPTION("Display Lighting Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GNU");






