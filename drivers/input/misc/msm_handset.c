/*
 * drivers/input/misc/msm_handset.c
 *
 * Driver for MSM handset events driver.
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/switch.h>

#include <mach/msm_handset.h>

static int ip_dev_reg;
static struct msm_handset *hs;

#define DRIVER_NAME	"msm-handset"

enum {
	NO_DEVICE	= 0,
	MSM_HEADSET	= 1,
};

static ssize_t msm_headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs->sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case MSM_HEADSET:
		return sprintf(buf, "Headset\n");
	}
	return -EINVAL;
}

static int __init hs_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ip_dev;

	hs = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	hs->sdev.name	= "h2w";
	hs->sdev.print_name = msm_headset_print_name;

	rc = switch_dev_register(&hs->sdev);
	if (rc)
		goto err_switch_dev_register;

	ip_dev = input_allocate_device();
	if (!ip_dev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ip_dev, hs);

	hs->ip_dev = ip_dev;

	ip_dev->name		= DRIVER_NAME;
	ip_dev->phys		= "msm-handset/input0";
	ip_dev->id.vendor	= 0x0001;
	ip_dev->id.product	= 1;
	ip_dev->id.version	= 1;

	ip_dev->evbit[0] = BIT(EV_SW) | BIT(EV_KEY);
	ip_dev->keybit[BIT_WORD(KEY_MEDIA)] = BIT_MASK(KEY_MEDIA);
	/* TODO: set this based on switchmap/keymap coming
	 * from board files.
	 */
	set_bit(SW_HEADPHONE_INSERT, ip_dev->swbit);

	rc = input_register_device(ip_dev);
	if (rc) {
		dev_err(&ip_dev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hs);
	ip_dev_reg = 1;

	return 0;

err_reg_input_dev:
	kfree(ip_dev);
err_alloc_input_dev:
	switch_dev_unregister(&hs->sdev);
err_switch_dev_register:
	kfree(hs);
	return rc;
}

static int hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hs = platform_get_drvdata(pdev);

	input_unregister_device(hs->ip_dev);
	kfree(hs->ip_dev);
	switch_dev_unregister(&hs->sdev);
	kfree(hs);
	ip_dev_reg = 0;
	return 0;
}

struct input_dev *msm_get_handset_input_dev(void)
{
	if (hs)
		return ip_dev_reg ? hs->ip_dev : NULL;
	else
		return NULL;
}
EXPORT_SYMBOL(msm_get_handset_input_dev);

struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device *hs_pdev;

static int __init hs_init(void)
{
	int rc;

	hs_pdev = platform_device_register_simple(DRIVER_NAME,
					-1, NULL, 0);

	if (IS_ERR(hs_pdev)) {
		rc = PTR_ERR(hs_pdev);
		goto err_reg_dev;
	}

	rc = platform_driver_register(&hs_driver);
	if (rc)
		goto err_reg_drv;

	return 0;

err_reg_drv:
	platform_device_unregister(hs_pdev);
err_reg_dev:
	return rc;
}
module_init(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
	platform_device_unregister(hs_pdev);
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm-handset");
