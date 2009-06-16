/*
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2009 Google, Inc.
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
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <linux/bu52014hfv.h>

struct bu52014hfv_info {
	unsigned char enabled;
	int    gpio_north;
	int    irq_north;
	int    gpio_south;
	int    irq_south;
	struct input_dev *idev;
	struct work_struct north_work;
	struct work_struct south_work;
	struct workqueue_struct *wq;
	unsigned int north_value;
	unsigned int south_value;
};

static int bu52014hfv_get_value(int gpio)
{
	return !gpio_get_value(gpio);
}

static void bu52014hfv_update(struct bu52014hfv_info *info, int gpio, int dock)
{
	int state;

	if (!info->enabled)
		return;
	state = bu52014hfv_get_value(gpio);
	input_report_switch(info->idev, dock, state);
	input_sync(info->idev);
}

void bu52014hfv_north_work_func(struct work_struct *work)
{
	struct bu52014hfv_info *info = container_of(work,
						    struct bu52014hfv_info,
						    north_work);
	bu52014hfv_update(info, info->gpio_north, info->north_value);
	enable_irq(info->irq_north);
}

void bu52014hfv_south_work_func(struct work_struct *work)
{
	struct bu52014hfv_info *info = container_of(work,
						    struct bu52014hfv_info,
						    south_work);
	bu52014hfv_update(info, info->gpio_south, info->south_value);
	enable_irq(info->irq_south);
}

static irqreturn_t bu52014hfv_isr(int irq, void *dev)
{
	struct bu52014hfv_info *info = dev;

	disable_irq_nosync(irq);

	if (irq == info->irq_north)
		queue_work(info->wq, &info->north_work);
	else if (irq == info->irq_south)
		queue_work(info->wq, &info->south_work);

	return IRQ_HANDLED;
}


static int __devinit bu52014hfv_probe(struct platform_device *pdev)
{
	struct bu52014hfv_platform_data *pdata = pdev->dev.platform_data;
	struct bu52014hfv_info *info;
	int ret = -1;

	info = kmalloc(sizeof(struct bu52014hfv_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		pr_err("%s: could not allocate space for module data: %d\n",
			__func__, ret);
		goto error_kmalloc_failed;
	}

	/* Initialize hall effect driver data */
	info->enabled = 0;
	info->gpio_north = pdata->docked_north_gpio;
	info->gpio_south = pdata->docked_south_gpio;

	info->irq_north = gpio_to_irq(pdata->docked_north_gpio);
	info->irq_south = gpio_to_irq(pdata->docked_south_gpio);

	if (pdata->north_is_desk) {
		info->north_value = SW_DOCK_DESK;
		info->south_value = SW_DOCK_CAR;
	} else {
		info->north_value = SW_DOCK_CAR;
		info->south_value = SW_DOCK_DESK;
	}

	info->wq = create_singlethread_workqueue("bu52014hfv_wq");
	if (!info->wq) {
		ret = -ENOMEM;
		pr_err("%s: cannot create work queue: %d\n", __func__, ret);
		goto error_create_wq_failed;
	}
	INIT_WORK(&info->north_work, bu52014hfv_north_work_func);
	INIT_WORK(&info->south_work, bu52014hfv_south_work_func);

	ret = request_irq(info->irq_north, bu52014hfv_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  BU52014HFV_MODULE_NAME, info);

	if (ret) {
		pr_err("%s: north request irq failed: %d\n", __func__, ret);
		goto error_request_irq_north_failed;
	}

	ret = request_irq(info->irq_south, bu52014hfv_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  BU52014HFV_MODULE_NAME, info);
	if (ret) {
		pr_err("%s: south request irq failed: %d\n", __func__, ret);
		goto error_request_irq_south_failed;
	}


	info->idev = input_allocate_device();
	if (!info->idev) {
		ret = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__, ret);
		goto error_input_allocate_failed;
	}

	info->idev->name = BU52014HFV_MODULE_NAME;
	set_bit(EV_SW, info->idev->evbit);
	set_bit(SW_DOCK_DESK, info->idev->swbit);
	set_bit(SW_DOCK_CAR, info->idev->swbit);

	platform_set_drvdata(pdev, info);

	ret = input_register_device(info->idev);
	if (ret) {
		pr_err("%s: input register device failed: %d\n", __func__, ret);
		goto error_input_register_failed;
	}

	info->enabled = 1;

	input_report_switch(info->idev,
			    info->north_value,
			    bu52014hfv_get_value(info->gpio_north));
	input_report_switch(info->idev,
			    info->south_value,
			    bu52014hfv_get_value(info->gpio_south));
	input_sync(info->idev);

	pr_info("%s: initialized N[%d, %d] S[%d, %d]\n",
		 __func__, info->gpio_north, info->irq_north,
		 info->gpio_south, info->irq_south);

	return 0;

error_input_register_failed:
	input_free_device(info->idev);
error_input_allocate_failed:
	free_irq(info->irq_south, info);
error_request_irq_south_failed:
	free_irq(info->irq_north, info);
error_request_irq_north_failed:
	destroy_workqueue(info->wq);
error_create_wq_failed:
	kfree(info);
error_kmalloc_failed:
	return ret;
}

static int __devexit bu52014hfv_remove(struct platform_device *pdev)
{
	struct bu52014hfv_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq_north, 0);
	free_irq(info->irq_south, 0);

	gpio_free(info->gpio_north);
	gpio_free(info->gpio_south);

	destroy_workqueue(info->wq);

	input_unregister_device(info->idev);
	input_free_device(info->idev);

	kfree(info);
	return 0;
}

static struct platform_driver bu52014hfv_driver = {
	.probe = bu52014hfv_probe,
	.remove = __devexit_p(bu52014hfv_remove),
	.driver = {
		.name = BU52014HFV_MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init bu52014hfv_os_init(void)
{
	return platform_driver_register(&bu52014hfv_driver);
}

static void __exit bu52014hfv_os_exit(void)
{
	platform_driver_unregister(&bu52014hfv_driver);
}

module_init(bu52014hfv_os_init);
module_exit(bu52014hfv_os_exit);

MODULE_DESCRIPTION("Rohm BU52014HFV Hall Effect Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
