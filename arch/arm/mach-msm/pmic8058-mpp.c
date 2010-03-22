/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 * Qualcomm PMIC8058 MPP driver
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include "gpio_chip.h"

#define PM8058_MPP_TO_INT(n) (PMIC8058_IRQ_BASE + NR_PMIC8058_GPIO_IRQS + (n))

static int pm8058_mpp_get_irq_num(struct qcom_gpio_chip *chip,
				   unsigned int gpio,
				   unsigned int *irqp,
				   unsigned long *irqnumflagsp)
{
	gpio -= chip->start;
	*irqp = PM8058_MPP_TO_INT(gpio);
	if (irqnumflagsp)
		*irqnumflagsp = 0;
	return 0;
}

static int pm8058_mpp_read(struct qcom_gpio_chip *chip, unsigned n)
{
	struct pm8058_chip	*pm_chip;

	n -= chip->start;
	pm_chip = dev_get_drvdata(chip->dev);
	return pm8058_mpp_get(pm_chip, n);
}

struct msm_gpio_chip pm8058_mpp_chip = {
	.chip = {
		.start = NR_GPIO_IRQS + NR_PMIC8058_GPIO_IRQS,
		.end = NR_GPIO_IRQS + NR_PMIC8058_GPIO_IRQS +
			NR_PMIC8058_MPP_IRQS - 1,
		.get_irq_num = pm8058_mpp_get_irq_num,
		.read = pm8058_mpp_read,
	}
};

static int __devinit pm8058_mpp_probe(struct platform_device *pdev)
{
	int	rc;

	pm8058_mpp_chip.chip.dev = &pdev->dev;
	rc = register_gpio_chip(&pm8058_mpp_chip.chip);
	pr_info("%s: register_gpio_chip(): rc=%d\n", __func__, rc);

	return rc;
}

static int __devexit pm8058_mpp_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver pm8058_mpp_driver = {
	.probe		= pm8058_mpp_probe,
	.remove		= __devexit_p(pm8058_mpp_remove),
	.driver		= {
		.name = "pm8058-mpp",
		.owner = THIS_MODULE,
	},
};

static int __init pm8058_mpp_init(void)
{
	return platform_driver_register(&pm8058_mpp_driver);
}

static void __exit pm8058_mpp_exit(void)
{
	platform_driver_unregister(&pm8058_mpp_driver);
}

subsys_initcall(pm8058_mpp_init);
module_exit(pm8058_mpp_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 MPP driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8058-mpp");

