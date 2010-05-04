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

#include <linux/bitmap.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <mach/msm_iomap.h>

/*
 * When a GPIO triggers, two separate decisions are made, controlled
 * by two separate flags.
 *
 * - First, INTR_RAW_STATUS_EN controls whether or not the GPIO_INTR_STATUS
 * register for that GPIO will be updated to reflect the triggering of that
 * gpio.  If this bit is 0, this register will not be updated.
 * - Second, INTR_ENABLE controls whether an interrupt is triggered.
 *
 * If INTR_ENABLE is set and INTR_RAW_STATUS_EN is NOT set, an interrupt
 * can be triggered but the status register will not reflect it.
 */

#define INTR_RAW_STATUS_EN (1 << 3)
#define INTR_DECT_CTL_EDGE (1 << 2)
#define INTR_POL_CTL_HI    (1 << 1)
#define INTR_ENABLE        1

#define TARGET_PROC_SCORPION 4
#define TARGET_PROC_NONE     7

#define DC_IRQ_DISABLE (0 << 3)
#define DC_IRQ_ENABLE  (1 << 3)

/*
 * There are some config flags used in the register which are not exposed
 * via our read/write-cfg API because their functionality is available via
 * other, generic means.
 */
#define PUBLIC_CFG_MASK \
	(MSM_GPIO_DRV_MASK | MSM_GPIO_FUNC_SEL_MASK | MSM_GPIO_PULL_MASK)

/*
 * There is no 'DC_POLARITY_LO' because the GIC is incapable
 * of asserting on falling edge or level-low conditions.  Even though
 * the registers allow for low-polarity inputs, the case can never arise.
 */
#define DC_POLARITY_HI (1 << 11)

#define GPIO_INTR_CFG_SU(gpio)    (MSM_TLMM_BASE + 0x0400 + (0x04 * (gpio)))
#define DIR_CONN_INTR_CFG_SU(irq) (MSM_TLMM_BASE + 0x0700 + (0x04 * (irq)))
#define GPIO_CFG(gpio)            (MSM_TLMM_BASE + 0x1000 + (0x10 * (gpio)))
#define GPIO_IN_OUT(gpio)         (MSM_TLMM_BASE + 0x1004 + (0x10 * (gpio)))
#define GPIO_INTR_CFG(gpio)       (MSM_TLMM_BASE + 0x1008 + (0x10 * (gpio)))
#define GPIO_INTR_STATUS(gpio)    (MSM_TLMM_BASE + 0x100c + (0x10 * (gpio)))
#define GPIO_OE_CLR(gpio)  (MSM_TLMM_BASE + 0x3100 + (0x04 * ((gpio) / 32)))
#define GPIO_OE_SET(gpio)  (MSM_TLMM_BASE + 0x3120 + (0x04 * ((gpio) / 32)))

/*
 * The 'enabled_irqs' bitmap is used to optimize the summary-irq handler.
 * By keeping track of which GPIOs have been activated as summary IRQs,
 * we avoid having to do readl calls on the INTR_STATUS registers of those
 * GPIOs which are not in use as interrupts.
 */
static DECLARE_BITMAP(enabled_irqs, NR_MSM_GPIOS);
static DEFINE_SPINLOCK(gpio_lock);

/*
 * "Banked" registers contain one bit for up to 32 gpios
 * keyed by the least-significant 5 bits in the GPIO's index.
 */
static inline void set_gpio_bit(unsigned gpio, void *addr)
{
	writel((0x01 << (gpio & 0x1f)), addr);
}

/*
 * It is important that the local bitfield not come out of sync with the
 * interrupt-enable bits in the GPIO config registers, so all enabling and
 * disabling of summary IRQs is done via these little helpers.  This also
 * helps keep the RAW_STATUS_EN bit in sync.
 */
static void enable_summary_irq(unsigned gpio)
{
	uint32_t intr_cfg;

	intr_cfg  = readl(GPIO_INTR_CFG(gpio));
	intr_cfg |= INTR_RAW_STATUS_EN | INTR_ENABLE;
	writel(intr_cfg, GPIO_INTR_CFG(gpio));
	set_bit(gpio, enabled_irqs);
}

static void disable_summary_irq(unsigned gpio)
{
	uint32_t intr_cfg;

	intr_cfg  = readl(GPIO_INTR_CFG(gpio));
	intr_cfg &= ~(INTR_RAW_STATUS_EN | INTR_ENABLE);
	writel(intr_cfg, GPIO_INTR_CFG(gpio));
	clear_bit(gpio, enabled_irqs);
}

int msm_gpio_install_direct_irq(unsigned gpio, unsigned irq)
{
	int rc = -EINVAL;
	unsigned long irq_flags;

	if (gpio < NR_MSM_GPIOS && irq < NR_TLMM_SCSS_DIR_CONN_IRQ) {
		spin_lock_irqsave(&gpio_lock, irq_flags);

		set_gpio_bit(gpio, GPIO_OE_CLR(gpio));
		disable_summary_irq(gpio);

		writel(DC_IRQ_ENABLE | TARGET_PROC_NONE,
			GPIO_INTR_CFG_SU(gpio));

		writel(DC_POLARITY_HI |	TARGET_PROC_SCORPION | (gpio << 3),
			DIR_CONN_INTR_CFG_SU(irq));

		spin_unlock_irqrestore(&gpio_lock, irq_flags);

		rc = 0;
	}

	return rc;
}
EXPORT_SYMBOL(msm_gpio_install_direct_irq);

void msm_gpio_write_cfg(unsigned gpio, uint32_t flags)
{
	unsigned long irq_flags;
	uint32_t      prev;
	uint32_t      next;

	if (gpio < NR_MSM_GPIOS) {
		spin_lock_irqsave(&gpio_lock, irq_flags);

		prev  = readl(GPIO_CFG(gpio));
		next  = prev & (1 << 9);
		next |= flags & PUBLIC_CFG_MASK;
		writel(next, GPIO_CFG(gpio));

		spin_unlock_irqrestore(&gpio_lock, irq_flags);
	}
}
EXPORT_SYMBOL(msm_gpio_write_cfg);

int msm_gpio_read_cfg(unsigned gpio, uint32_t *flags)
{
	int rc = -EINVAL;

	if (gpio < NR_MSM_GPIOS && flags != NULL) {
		*flags  = readl(GPIO_CFG(gpio));
		*flags &= PUBLIC_CFG_MASK;
		rc = 0;
	}

	return rc;
}
EXPORT_SYMBOL(msm_gpio_read_cfg);

static int msm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int i = 0;

	if (offset < NR_MSM_GPIOS)
		i = (readl(GPIO_IN_OUT(offset)) & 0x03);

	return i;
}

static void msm_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	if (offset < NR_MSM_GPIOS)
		writel((val ? 0x02 : 0x00), GPIO_IN_OUT(offset));
}

static int msm_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	int i = -EINVAL;
	if (offset < NR_MSM_GPIOS) {
		set_gpio_bit(offset, GPIO_OE_CLR(offset));
		i = 0;
	}
	return i;
}

static int msm_gpio_direction_output(struct gpio_chip *chip,
				unsigned offset,
				int val)
{
	int i = -EINVAL;
	if (offset < NR_MSM_GPIOS) {
		msm_gpio_set(chip, offset, val);
		set_gpio_bit(offset, GPIO_OE_SET(offset));
		i = 0;
	}
	return i;
}

static struct gpio_chip msm_gpios = {
	.label            = "MSM_GPIO",
	.direction_input  = msm_gpio_direction_input,
	.direction_output = msm_gpio_direction_output,
	.get              = msm_gpio_get,
	.set              = msm_gpio_set,
	.base             = 0,
	.ngpio            = NR_MSM_GPIOS
};

static void msm_gpio_irq_enable(unsigned int irq)
{
	unsigned long irq_flags;
	unsigned      gpio = irq_to_gpio(irq);

	spin_lock_irqsave(&gpio_lock, irq_flags);
	set_gpio_bit(gpio, GPIO_OE_CLR(gpio));
	writel(DC_IRQ_DISABLE | TARGET_PROC_SCORPION, GPIO_INTR_CFG_SU(gpio));
	enable_summary_irq(gpio);
	spin_unlock_irqrestore(&gpio_lock, irq_flags);
}

static void msm_gpio_irq_disable(unsigned int irq)
{
	unsigned long irq_flags;
	unsigned      gpio = irq_to_gpio(irq);

	spin_lock_irqsave(&gpio_lock, irq_flags);
	disable_summary_irq(gpio);
	writel(DC_IRQ_DISABLE | TARGET_PROC_NONE, GPIO_INTR_CFG_SU(gpio));
	spin_unlock_irqrestore(&gpio_lock, irq_flags);
}

static void msm_gpio_irq_ack(unsigned int irq)
{
	writel(0x01, GPIO_INTR_STATUS(irq_to_gpio(irq)));
}

static void msm_gpio_irq_mask(unsigned int irq)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&gpio_lock, irq_flags);
	disable_summary_irq(irq_to_gpio(irq));
	spin_unlock_irqrestore(&gpio_lock, irq_flags);
}

static void msm_gpio_irq_mask_ack(unsigned int irq)
{
	msm_gpio_irq_mask(irq);
	msm_gpio_irq_ack(irq);
}

static void msm_gpio_irq_unmask(unsigned int irq)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&gpio_lock, irq_flags);
	enable_summary_irq(irq_to_gpio(irq));
	spin_unlock_irqrestore(&gpio_lock, irq_flags);
}

static int msm_gpio_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	uint32_t      bits;
	unsigned long irq_flags;
	void         *addr = GPIO_INTR_CFG(irq_to_gpio(irq));

	spin_lock_irqsave(&gpio_lock, irq_flags);

	bits = readl(addr);

	if (flow_type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
		bits |= INTR_DECT_CTL_EDGE;
	else
		bits &= ~INTR_DECT_CTL_EDGE;

	if (flow_type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_LEVEL_HIGH))
		bits |= INTR_POL_CTL_HI;
	else
		bits &= ~INTR_POL_CTL_HI;

	writel(bits, addr);

	spin_unlock_irqrestore(&gpio_lock, irq_flags);

	return 0;
}

/*
 * When the summary IRQ is raised, any number of GPIO lines may be high.
 * It is the job of the summary handler to find all those GPIO lines
 * which have been set as summary IRQ lines and which are triggered,
 * and to call their interrupt handlers.
 */
static void msm_summary_irq_handler(unsigned int irq,
				struct irq_desc *desc)
{
	unsigned long i;

	for (i = find_first_bit(enabled_irqs, NR_MSM_GPIOS);
	     i < NR_MSM_GPIOS;
	     i = find_next_bit(enabled_irqs, NR_MSM_GPIOS, i + 1)) {
		if (readl(GPIO_INTR_STATUS(i)) & 0x01) {
			writel(0x01, GPIO_INTR_STATUS(i));
			generic_handle_irq(gpio_to_irq(i));
		}
	}
	desc->chip->ack(irq);
}

struct irq_chip msm_summary_irq_chip = {
	.name         = "MSM_GPIO",
	.enable       = msm_gpio_irq_enable,
	.disable      = msm_gpio_irq_disable,
	.ack          = msm_gpio_irq_ack,
	.mask         = msm_gpio_irq_mask,
	.unmask       = msm_gpio_irq_unmask,
	.mask_ack     = msm_gpio_irq_mask_ack,
	.set_type     = msm_gpio_irq_set_type
};

static int __init msm_gpio_init(void)
{
	int i;

	bitmap_zero(enabled_irqs, NR_MSM_GPIOS);
	gpiochip_add(&msm_gpios);
	for (i = 0; i < NR_MSM_GPIOS; ++i) {
		set_irq_chip(gpio_to_irq(i), &msm_summary_irq_chip);
		set_irq_handler(gpio_to_irq(i), handle_edge_irq);
		set_irq_flags(gpio_to_irq(i), IRQF_VALID);
	}

	set_irq_chained_handler(TLMM_SCSS_SUMMARY_IRQ,
				msm_summary_irq_handler);

	return 0;
}
arch_initcall(msm_gpio_init);
