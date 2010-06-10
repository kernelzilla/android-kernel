/*
 * Driver for Qualcomm MSM7200a and related SoC GPIO.
 * Supported chipset families include:
 * MSM7x01(a), MSM7x25, MSM7x27, MSM7x30, QSD8x50(a)
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/msm7200a-gpio.h>

struct msm_gpio_dev {
	struct gpio_chip		gpio_chip;
	spinlock_t			lock;
	struct msm7200a_gpio_regs	regs;
};

#define TO_MSM_GPIO_DEV(c) container_of(c, struct msm_gpio_dev, gpio_chip)

static inline unsigned bit(unsigned offset)
{
	BUG_ON(offset >= sizeof(unsigned) * 8);
	return 1U << offset;
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void set_gpio_bit(unsigned n, void __iomem *reg)
{
	writel(readl(reg) | bit(n), reg);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void clr_gpio_bit(unsigned n, void __iomem *reg)
{
	writel(readl(reg) & ~bit(n), reg);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void
msm_gpio_write(struct msm_gpio_dev *dev, unsigned n, unsigned on)
{
	if (on)
		set_gpio_bit(n, dev->regs.out);
	else
		clr_gpio_bit(n, dev->regs.out);
}

static int gpio_chip_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	clr_gpio_bit(offset, msm_gpio->regs.oe);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int
gpio_chip_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);

	msm_gpio_write(msm_gpio, offset, value);
	set_gpio_bit(offset, msm_gpio->regs.oe);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int gpio_chip_get(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;
	int ret;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	ret = readl(msm_gpio->regs.in) & bit(offset) ? 1 : 0;
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return ret;
}

static void gpio_chip_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_dev *msm_gpio = TO_MSM_GPIO_DEV(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	msm_gpio_write(msm_gpio, offset, value);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);
}

static int msm_gpio_probe(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio;
	struct msm7200a_gpio_platform_data *pdata =
		(struct msm7200a_gpio_platform_data *)dev->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	msm_gpio = kzalloc(sizeof(struct msm_gpio_dev), GFP_KERNEL);
	if (!msm_gpio)
		return -ENOMEM;

	spin_lock_init(&msm_gpio->lock);
	platform_set_drvdata(dev, msm_gpio);
	memcpy(&msm_gpio->regs,
	       &pdata->regs,
	       sizeof(struct msm7200a_gpio_regs));

	msm_gpio->gpio_chip.label            = dev->name;
	msm_gpio->gpio_chip.base             = pdata->gpio_base;
	msm_gpio->gpio_chip.ngpio            = pdata->ngpio;
	msm_gpio->gpio_chip.direction_input  = gpio_chip_direction_input;
	msm_gpio->gpio_chip.direction_output = gpio_chip_direction_output;
	msm_gpio->gpio_chip.get              = gpio_chip_get;
	msm_gpio->gpio_chip.set              = gpio_chip_set;

	ret = gpiochip_add(&msm_gpio->gpio_chip);
	if (ret < 0)
		goto err;

	return ret;
err:
	kfree(msm_gpio);
	return ret;
}

static int msm_gpio_remove(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(dev);
	int ret = gpiochip_remove(&msm_gpio->gpio_chip);

	if (ret == 0)
		kfree(msm_gpio);

	return ret;
}

static struct platform_driver msm_gpio_driver = {
	.probe = msm_gpio_probe,
	.remove = msm_gpio_remove,
	.driver = {
		.name = "msm7200a-gpio",
		.owner = THIS_MODULE,
	},
};

static int __init msm_gpio_init(void)
{
	return platform_driver_register(&msm_gpio_driver);
}

static void __exit msm_gpio_exit(void)
{
	platform_driver_unregister(&msm_gpio_driver);
}

postcore_initcall(msm_gpio_init);
module_exit(msm_gpio_exit);

MODULE_DESCRIPTION("Driver for Qualcomm MSM 7200a-family SoC GPIOs");
MODULE_LICENSE("GPLv2");
