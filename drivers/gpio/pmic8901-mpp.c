/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*
 * Qualcomm PMIC8901 MPP driver
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8901.h>
#include <mach/mpp.h>

/* MPP Control Registers */
#define	SSBI_MPP_CNTRL_BASE		0x27
#define	SSBI_MPP_CNTRL(n)		(SSBI_MPP_CNTRL_BASE + (n))

/* MPP Type */
#define	PM8901_MPP_TYPE_MASK		0xE0
#define	PM8901_MPP_TYPE_SHIFT		5

/* MPP Config Level */
#define	PM8901_MPP_CONFIG_LVL_MASK	0x1C
#define	PM8901_MPP_CONFIG_LVL_SHIFT	2

/* MPP Config Control */
#define	PM8901_MPP_CONFIG_CTL_MASK	0x03

static int pm8901_mpp_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct pm8901_gpio_platform_data *pdata;
	pdata = chip->dev->platform_data;
	return pdata->irq_base + offset;
}

static int pm8901_mpp_read(struct gpio_chip *chip, unsigned offset)
{
	struct pm8901_chip *pm_chip;
	pm_chip = dev_get_drvdata(chip->dev);
	return pm8901_mpp_get(pm_chip, offset);
}

static struct gpio_chip pm8901_mpp_chip = {
	.label		= "pm8901-mpp",
	.to_irq		= pm8901_mpp_to_irq,
	.get		= pm8901_mpp_read,
	.ngpio		= PM8901_MPPS,
};

int pm8901_mpp_config(unsigned mpp, unsigned type, unsigned level,
		      unsigned control)
{
	u8	config;
	int	rc;
	struct pm8901_chip *pm_chip;

	if (mpp >= PM8901_MPPS)
		return -EINVAL;

	pm_chip = dev_get_drvdata(pm8901_mpp_chip.dev);

	config = (type << PM8901_MPP_TYPE_SHIFT) & PM8901_MPP_TYPE_MASK;
	config |= (level << PM8901_MPP_CONFIG_LVL_SHIFT) &
			PM8901_MPP_CONFIG_LVL_MASK;
	config |= control & PM8901_MPP_CONFIG_CTL_MASK;

	rc = pm8901_write(pm_chip, SSBI_MPP_CNTRL(mpp), &config, 1);
	if (rc)
		pr_err("%s: pm8901_write(): rc=%d\n", __func__, rc);

	return rc;
}
EXPORT_SYMBOL(pm8901_mpp_config);

static int __devinit pm8901_mpp_probe(struct platform_device *pdev)
{
	int ret;
	struct pm8901_gpio_platform_data *pdata = pdev->dev.platform_data;

	pm8901_mpp_chip.dev = &pdev->dev;
	pm8901_mpp_chip.base = pdata->gpio_base;
	ret = gpiochip_add(&pm8901_mpp_chip);
	pr_info("%s: gpiochip_add(): rc=%d\n", __func__, ret);
	return ret;
}

static int __devexit pm8901_mpp_remove(struct platform_device *pdev)
{
	return gpiochip_remove(&pm8901_mpp_chip);
}

static struct platform_driver pm8901_mpp_driver = {
	.probe		= pm8901_mpp_probe,
	.remove		= __devexit_p(pm8901_mpp_remove),
	.driver		= {
		.name = "pm8901-mpp",
		.owner = THIS_MODULE,
	},
};

static int __init pm8901_mpp_init(void)
{
	return platform_driver_register(&pm8901_mpp_driver);
}

static void __exit pm8901_mpp_exit(void)
{
	platform_driver_unregister(&pm8901_mpp_driver);
}

subsys_initcall(pm8901_mpp_init);
module_exit(pm8901_mpp_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8901 MPP driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8901-mpp");
