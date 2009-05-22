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

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>
#include <linux/uaccess.h>

#include <linux/sfh7743.h>

struct sfh7743_data {
	int enabled;
	int gpio_intr;
	int irq;
	struct mutex lock;
	struct work_struct wq;
	struct workqueue_struct *working_queue;
	struct input_dev *idev;
	int (*power_on)(void);
	int (*power_off)(void);
};

static irqreturn_t sfh7743_irq_handler(int irq, void *dev)
{
	struct sfh7743_data *info = dev;

	disable_irq_nosync(irq);
	queue_work(info->working_queue, &info->wq);

	return IRQ_HANDLED;
}

static void sfh7743_irq_bottom_half(struct work_struct *work)
{
	int distance;

	struct sfh7743_data *info = container_of(work,
						 struct sfh7743_data,
						 wq);

	if (info->enabled == SFH7743_ENABLED) {
		if (gpio_get_value(info->gpio_intr))
			distance = SFH7743_PROXIMITY_NEAR;
		else
			distance = SFH7743_PROXIMITY_FAR;

		input_report_abs(info->idev, ABS_DISTANCE, distance);
		input_sync(info->idev);
	} else {
		pr_info("%s: Not enabled\n", __func__);
	}
	enable_irq(info->irq);
}

int sfh7743_input_open(struct input_dev *input)
{
	struct sfh7743_data *info = input_get_drvdata(input);

	mutex_lock(&info->lock);
	if (info->power_on) {
		info->power_on();
		info->enabled = SFH7743_ENABLED;
	} else
		pr_err("%s: Power on func is NULL\n", __func__);

	mutex_unlock(&info->lock);
	return 0;
}

void sfh7743_input_close(struct input_dev *dev)
{
	struct sfh7743_data *info = input_get_drvdata(dev);

	mutex_lock(&info->lock);
	if (info->power_off) {
		info->power_off();
		info->enabled = SFH7743_DISABLED;
	} else
		pr_err("%s: Power off func is NULL\n", __func__);

	mutex_unlock(&info->lock);
}

static int __devexit sfh7743_remove(struct platform_device *pdev)
{
	struct sfh7743_data *info = platform_get_drvdata(pdev);

	if (info->power_off) {
		info->power_off();
		info->enabled = SFH7743_DISABLED;
	}

	if (info->irq != -1)
		free_irq(info->irq, 0);

	if (info->gpio_intr != -1)
		gpio_free(info->gpio_intr);

	if (info->idev) {
		input_unregister_device(info->idev);
		input_free_device(info->idev);
	}

	return 0;
}

static int sfh7743_probe(struct platform_device *pdev)
{
	int error = 0;
	struct sfh7743_platform_data *pdata = pdev->dev.platform_data;
	struct sfh7743_data *info;

	info = kmalloc(sizeof(struct sfh7743_data), GFP_KERNEL);
	if (!info) {
		pr_err("%s: Could not allocate space for module data",
		       __func__);
		return -ENOMEM;
	}

	info->gpio_intr = pdata->gpio_prox_int;
	info->power_on = pdata->power_on;
	info->power_off = pdata->power_off;
	info->irq = gpio_to_irq(info->gpio_intr);

	info->working_queue = create_singlethread_workqueue("sfh7743_wq");
	INIT_WORK(&info->wq, sfh7743_irq_bottom_half);

	error = request_irq(info->irq, sfh7743_irq_handler,
			    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			    SFH7743_MODULE_NAME, info);
	if (error) {
		pr_err("%s: request_irq failed: %d\n", __func__, error);
		goto exit_request_irq_failed;
	}

	info->idev = input_allocate_device();
	if (!info->idev) {
		pr_err("%s: Insufficient memory for input device\n",
			__func__);
		error = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}

	mutex_init(&info->lock);

	info->idev->name = SFH7743_MODULE_NAME;
	info->idev->open = sfh7743_input_open;
	info->idev->close = sfh7743_input_close;

	input_set_drvdata(info->idev, info);

	set_bit(EV_ABS, info->idev->evbit);
	set_bit(ABS_DISTANCE, info->idev->absbit);

	error = input_register_device(info->idev);
	if (error != 0) {
		pr_err("%s: Failed to register input device\n",
			__func__);
		goto exit_input_register_device_failed;
	}

	if (info->power_on)
		info->power_on();
	info->enabled = SFH7743_ENABLED;

	disable_irq_nosync(info->irq);
	queue_work(info->working_queue, &info->wq);

	return 0;

exit_input_register_device_failed:
	input_free_device(info->idev);
	info->idev = NULL;
exit_input_dev_alloc_failed:
	free_irq(info->irq, 0);
exit_request_irq_failed:
	kfree(info);
	return error;
}

static struct platform_driver sfh7743_driver = {
	.probe = sfh7743_probe,
	.remove = __devexit_p(sfh7743_remove),
	.driver = {
		   .name = SFH7743_MODULE_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init sfh7743_init(void)
{
	return platform_driver_register(&sfh7743_driver);
}

static void __exit sfh7743_exit(void)
{
	platform_driver_unregister(&sfh7743_driver);
}

module_init(sfh7743_init);
module_exit(sfh7743_exit);

MODULE_DESCRIPTION("IR proximity device driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
