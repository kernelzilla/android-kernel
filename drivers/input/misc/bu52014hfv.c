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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>

#include <linux/bu52014hfv.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

struct bu52014hfv_info {
	int gpio_north;
	int gpio_south;

	int irq_north;
	int irq_south;

	struct work_struct irq_north_work;
	struct work_struct irq_south_work;

	struct workqueue_struct *work_queue;
	struct switch_dev sdev;

	unsigned int north_value;
	unsigned int south_value;
};
enum {
	NO_DOCK,
	DESK_DOCK,
	CAR_DOCK,
};

static ssize_t print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case NO_DOCK:
		return sprintf(buf, "None\n");
	case DESK_DOCK:
		return sprintf(buf, "DESK\n");
	case CAR_DOCK:
		return sprintf(buf, "CAR\n");
	}

	return -EINVAL;
}
static int bu52014hfv_update(struct bu52014hfv_info *info, int gpio, int dock)
{
	int state = !gpio_get_value(gpio);

	if (state)
		switch_set_state(&info->sdev, dock);
	else
		switch_set_state(&info->sdev, NO_DOCK);

	return state;
}

void bu52014hfv_irq_north_work_func(struct work_struct *work)
{
	struct bu52014hfv_info *info = container_of(work,
						    struct bu52014hfv_info,
						    irq_north_work);
	bu52014hfv_update(info, info->gpio_north, info->north_value);
	enable_irq(info->irq_north);
}

void bu52014hfv_irq_south_work_func(struct work_struct *work)
{
	struct bu52014hfv_info *info = container_of(work,
						    struct bu52014hfv_info,
						    irq_south_work);
	bu52014hfv_update(info, info->gpio_south, info->south_value);
	enable_irq(info->irq_south);
}

static irqreturn_t bu52014hfv_isr(int irq, void *dev)
{
	struct bu52014hfv_info *info = dev;

	disable_irq_nosync(irq);

	if (irq == info->irq_north)
		queue_work(info->work_queue, &info->irq_north_work);
	else if (irq == info->irq_south)
		queue_work(info->work_queue, &info->irq_south_work);

	return IRQ_HANDLED;
}

static int __devinit bu52014hfv_probe(struct platform_device *pdev)
{
	struct bu52014hfv_platform_data *pdata = pdev->dev.platform_data;
	struct bu52014hfv_info *info;
	int ret = -1;

	info = kzalloc(sizeof(struct bu52014hfv_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		pr_err("%s: could not allocate space for module data: %d\n",
		       __func__, ret);
		goto error_kmalloc_failed;
	}

	/* Initialize hall effect driver data */
	info->gpio_north = pdata->docked_north_gpio;
	info->gpio_south = pdata->docked_south_gpio;

	info->irq_north = gpio_to_irq(pdata->docked_north_gpio);
	info->irq_south = gpio_to_irq(pdata->docked_south_gpio);

	if (pdata->north_is_desk) {
		info->north_value = DESK_DOCK;
		info->south_value = CAR_DOCK;
	} else {
		info->north_value = CAR_DOCK;
		info->south_value = DESK_DOCK;
	}

	info->work_queue = create_singlethread_workqueue("bu52014hfv_wq");
	if (!info->work_queue) {
		ret = -ENOMEM;
		pr_err("%s: cannot create work queue: %d\n", __func__, ret);
		goto error_create_wq_failed;
	}
	INIT_WORK(&info->irq_north_work, bu52014hfv_irq_north_work_func);
	INIT_WORK(&info->irq_south_work, bu52014hfv_irq_south_work_func);

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

	enable_irq_wake(info->irq_north);
	enable_irq_wake(info->irq_south);

	info->sdev.name = "dock";
	info->sdev.print_name = print_name;
	ret = switch_dev_register(&info->sdev);
	if (ret) {
		pr_err("%s: error registering switch device %d\n",
			__func__, ret);
		goto error_switch_device_failed;
	}
	platform_set_drvdata(pdev, info);

	ret = bu52014hfv_update(info, info->gpio_south, info->south_value);
	if (!ret)
		bu52014hfv_update(info, info->gpio_north, info->north_value);

	return 0;

error_switch_device_failed:
	free_irq(info->irq_south, info);
error_request_irq_south_failed:
	free_irq(info->irq_north, info);
error_request_irq_north_failed:
	destroy_workqueue(info->work_queue);
error_create_wq_failed:
	kfree(info);
error_kmalloc_failed:
	return ret;
}

static int __devexit bu52014hfv_remove(struct platform_device *pdev)
{
	struct bu52014hfv_info *info = platform_get_drvdata(pdev);

	disable_irq_wake(info->irq_north);
	disable_irq_wake(info->irq_south);

	free_irq(info->irq_north, 0);
	free_irq(info->irq_south, 0);

	gpio_free(info->gpio_north);
	gpio_free(info->gpio_south);

	destroy_workqueue(info->work_queue);

	switch_dev_unregister(&info->sdev);

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

#ifdef CONFIG_ARM_OF
static int halleffect_of_init(void)
{
	int device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_HALLEFFECT_DOCK);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_HALLEFFECT_DOCK);
		return -ENODEV;
	}

	prop = of_get_property(node, "device_available", NULL);
	if (prop)
		device_available = (int) *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", DT_PROP_DEV_AVAILABLE);
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}
#endif

static int __init bu52014hfv_os_init(void)
{
#ifdef CONFIG_ARM_OF
	int available = halleffect_of_init();
	if (available != 1) {
		pr_err("Cannot configure bu52014hfv device. Reason =  %d\n",
			available);
		return -1;
	}
#endif
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
