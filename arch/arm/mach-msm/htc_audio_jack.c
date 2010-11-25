/*
 *  3.5mm audio jack detection driver.
 *
 *  Copyright (C) 2008 htc, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <mach/htc_headset.h>
#include <mach/audio_jack.h>
#include <asm/gpio.h>


/* #define DEBUG */

#define MODULE_NAME	"audio_jack"

#ifdef DEBUG
#define AJ_DBG(fmt, arg...) \
	printk(KERN_INFO "[Audio Jack] %s " fmt "\r\n", __func__, ## arg)
#else
#define AJ_DBG(fmt, arg...) do {} while (0)
#endif

struct audio_jack_info {
	unsigned int irq_jack;
	int audio_jack_detect;
	int audio_jack_flag;

	struct hrtimer detection_timer;
	ktime_t debounce_time;
};

static struct audio_jack_info *pjack_info;


static enum hrtimer_restart detect_35mm_event_timer_func(struct hrtimer *data)
{
#ifdef CONFIG_HTC_HEADSET
	int state;

	AJ_DBG("");
	state = !gpio_get_value(pjack_info->audio_jack_detect);
	if (pjack_info->audio_jack_flag != state) {
		pjack_info->audio_jack_flag = state;
		extended_headset(pjack_info->audio_jack_flag);
	}

#endif
	return HRTIMER_NORESTART;
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	AJ_DBG("");
	do {
		value1 = gpio_get_value(pjack_info->audio_jack_detect);
		set_irq_type(pjack_info->irq_jack, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(pjack_info->audio_jack_detect);
	} while (value1 != value2 && retry_limit-- > 0);

	AJ_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	if ((pjack_info->audio_jack_flag == 0) ^ value2) {
		/* Do the rest of the work in timer context */
		hrtimer_start(&pjack_info->detection_timer,
			      pjack_info->debounce_time,
			      HRTIMER_MODE_REL);
	}
	return IRQ_HANDLED;
}


static int audiojack_probe(struct platform_device *pdev)
{
	int ret;
	struct audio_jack_platform_data *pdata = pdev->dev.platform_data;

	printk(KERN_ERR "Audiojack: Registering in audiojack_probe\n");
	pjack_info = kzalloc(sizeof(struct audio_jack_info), GFP_KERNEL);
	if (!pjack_info)
		return -ENOMEM;

	pjack_info->audio_jack_detect = pdata->gpio;
	pjack_info->audio_jack_flag = 0;

	/* 200 ms */
	pjack_info->debounce_time = ktime_set(0, 200000000);
	hrtimer_init(&pjack_info->detection_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pjack_info->detection_timer.function = detect_35mm_event_timer_func;

	ret = gpio_request(pjack_info->audio_jack_detect, "3.5mm_detect");
	if (ret < 0)
		goto err_request_detect_gpio;

	ret = gpio_direction_input(pjack_info->audio_jack_detect);
	if (ret < 0)
		goto err_set_detect_gpio;

	pjack_info->irq_jack = gpio_to_irq(pjack_info->audio_jack_detect);
	if (pjack_info->irq_jack < 0) {
		ret = pjack_info->irq_jack;
		goto err_request_detect_irq;
	}

	ret = request_irq(pjack_info->irq_jack,
			  detect_irq_handler,
			  IRQF_TRIGGER_LOW, "35mm_headset", NULL);
	if (ret < 0)
		goto err_request_detect_irq;

	ret = set_irq_wake(pjack_info->irq_jack, 1);
	if (ret < 0)
		goto err_set_irq_wake;

	return 0;


err_set_irq_wake:
	free_irq(pjack_info->irq_jack, 0);
err_request_detect_irq:
err_set_detect_gpio:
	gpio_free(pjack_info->audio_jack_detect);
err_request_detect_gpio:
	printk(KERN_ERR "Audiojack: Failed in audiojack_probe\n");

	return ret;
}

static int audiojack_remove(struct platform_device *pdev)
{
	free_irq(pjack_info->irq_jack, 0);
	gpio_free(pjack_info->audio_jack_detect);

	return 0;
}


static struct platform_driver audiojack_driver = {
	.probe		= audiojack_probe,
	.remove		= audiojack_remove,
	.driver		= {
		.name		= MODULE_NAME,
		.owner		= THIS_MODULE,
	},
};


static int __init audiojack_init(void)
{
	return platform_driver_register(&audiojack_driver);
}

static void __exit audiojack_exit(void)
{
	platform_driver_unregister(&audiojack_driver);
}

module_init(audiojack_init);
module_exit(audiojack_exit);

MODULE_AUTHOR("Farmer Tseng <farmer_tseng@htc.com>");
MODULE_DESCRIPTION("audio jack detection driver");
MODULE_LICENSE("GPL");
