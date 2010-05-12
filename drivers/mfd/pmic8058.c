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
 * Qualcomm PMIC8058 driver
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/mfd/pmic8058.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <asm-generic/gpio.h>

/* PMIC8058 Revision */
#define SSBI_REG_REV			0x002  /* PMIC4 revision */

/* PMIC8058 IRQ */
#define	SSBI_REG_ADDR_IRQ_BASE		0x1BB

#define	SSBI_REG_ADDR_IRQ_ROOT		(SSBI_REG_ADDR_IRQ_BASE + 0)
#define	SSBI_REG_ADDR_IRQ_M_STATUS1	(SSBI_REG_ADDR_IRQ_BASE + 1)
#define	SSBI_REG_ADDR_IRQ_M_STATUS2	(SSBI_REG_ADDR_IRQ_BASE + 2)
#define	SSBI_REG_ADDR_IRQ_M_STATUS3	(SSBI_REG_ADDR_IRQ_BASE + 3)
#define	SSBI_REG_ADDR_IRQ_M_STATUS4	(SSBI_REG_ADDR_IRQ_BASE + 4)
#define	SSBI_REG_ADDR_IRQ_BLK_SEL	(SSBI_REG_ADDR_IRQ_BASE + 5)
#define	SSBI_REG_ADDR_IRQ_IT_STATUS	(SSBI_REG_ADDR_IRQ_BASE + 6)
#define	SSBI_REG_ADDR_IRQ_CONFIG	(SSBI_REG_ADDR_IRQ_BASE + 7)
#define	SSBI_REG_ADDR_IRQ_RT_STATUS	(SSBI_REG_ADDR_IRQ_BASE + 8)

#define	PM8058_IRQF_LVL_SEL		0x01	/* level select */
#define	PM8058_IRQF_MASK_FE		0x02	/* mask falling edge */
#define	PM8058_IRQF_MASK_RE		0x04	/* mask rising edge */
#define	PM8058_IRQF_CLR			0x08	/* clear interrupt */
#define	PM8058_IRQF_BITS_MASK		0x70
#define	PM8058_IRQF_BITS_SHIFT		4
#define	PM8058_IRQF_WRITE		0x80

#define	PM8058_IRQF_MASK_ALL		(PM8058_IRQF_MASK_FE | \
					PM8058_IRQF_MASK_RE)
#define PM8058_IRQF_W_C_M		(PM8058_IRQF_WRITE |	\
					PM8058_IRQF_CLR |	\
					PM8058_IRQF_MASK_ALL)

/* GPIO registers */
#define	SSBI_REG_ADDR_GPIO_BASE		0x150
#define	SSBI_REG_ADDR_GPIO(n)		(SSBI_REG_ADDR_GPIO_BASE + n)

/* GPIO */
#define	PM8058_GPIO_BANK_MASK		0x70
#define	PM8058_GPIO_BANK_SHIFT		4
#define	PM8058_GPIO_WRITE		0x80

/* Bank 0 */
#define	PM8058_GPIO_VIN_MASK		0x0E
#define	PM8058_GPIO_VIN_SHIFT		1
#define	PM8058_GPIO_MODE_ENABLE		0x01

/* Bank 1 */
#define	PM8058_GPIO_MODE_MASK		0x0C
#define	PM8058_GPIO_MODE_SHIFT		2
#define	PM8058_GPIO_OUT_BUFFER		0x02
#define	PM8058_GPIO_OUT_INVERT		0x01

#define	PM8058_GPIO_MODE_OFF		3
#define	PM8058_GPIO_MODE_OUTPUT		2
#define	PM8058_GPIO_MODE_INPUT		0
#define	PM8058_GPIO_MODE_BOTH		1

/* Bank 2 */
#define	PM8058_GPIO_PULL_MASK		0x0E
#define	PM8058_GPIO_PULL_SHIFT		1

/* Bank 3 */
#define	PM8058_GPIO_OUT_STRENGTH_MASK		0x0C
#define	PM8058_GPIO_OUT_STRENGTH_SHIFT		2

/* Bank 4 */
#define	PM8058_GPIO_FUNC_MASK		0x0E
#define	PM8058_GPIO_FUNC_SHIFT		1

/* Bank 5 */
#define	PM8058_GPIO_NON_INT_POL_INV	0x08
#define PM8058_GPIO_BANKS		6

#define	MAX_PM_IRQ		256
#define	MAX_PM_BLOCKS		(MAX_PM_IRQ / 8 + 1)
#define	MAX_PM_MASTERS		(MAX_PM_BLOCKS / 8 + 1)

struct pm8058_chip {
	struct pm8058_platform_data	pdata;

	struct i2c_client		*dev;

	u8	irqs_allowed[MAX_PM_BLOCKS];
	u8	blocks_allowed[MAX_PM_MASTERS];
	u8	masters_allowed;
	int	pm_max_irq;
	int	pm_max_blocks;
	int	pm_max_masters;

	u8	config[MAX_PM_IRQ];
	u8	wake_enable[MAX_PM_IRQ];
	u16	count_wakeable;

	u8	revision;

	u8	gpio_bank1[PM8058_GPIOS];
	spinlock_t	pm_lock;
};

static struct pm8058_chip *pmic_chip;

/* Helper Functions */
DEFINE_RATELIMIT_STATE(pm8058_msg_ratelimit, 60 * HZ, 10);

static inline int pm8058_can_print(void)
{
	return __ratelimit(&pm8058_msg_ratelimit);
}

static inline int
ssbi_write(struct i2c_client *client, u16 addr, const u8 *buf, size_t len)
{
	int	rc;
	struct	i2c_msg msg = {
		.addr           = addr,
		.flags          = 0x0,
		.buf            = (u8 *)buf,
		.len            = len,
	};

	rc = i2c_transfer(client->adapter, &msg, 1);
	return (rc == 1) ? 0 : rc;
}

static inline int
ssbi_read(struct i2c_client *client, u16 addr, u8 *buf, size_t len)
{
	int	rc;
	struct	i2c_msg msg = {
		.addr           = addr,
		.flags          = I2C_M_RD,
		.buf            = buf,
		.len            = len,
	};

	rc = i2c_transfer(client->adapter, &msg, 1);
	return (rc == 1) ? 0 : rc;
}

/* External APIs */
int pm8058_rev(struct pm8058_chip *chip)
{
	if (chip == NULL)
		return -EINVAL;

	return chip->revision;
}
EXPORT_SYMBOL(pm8058_rev);

int pm8058_irq_get_rt_status(struct pm8058_chip *chip, int irq)
{
	int     rc;
	u8      block, bits, bit;
	unsigned long   irqsave;

	if (chip == NULL || irq < chip->pdata.irq_base ||
			irq >= chip->pdata.irq_base + MAX_PM_IRQ)
		return -EINVAL;

	irq -= chip->pdata.irq_base;

	block = irq / 8;
	bit = irq % 8;

	spin_lock_irqsave(&chip->pm_lock, irqsave);

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_BLK_SEL, &block, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(): rc=%d (Select Block)\n",
				__func__, rc);
		goto bail_out;
	}

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_RT_STATUS, &bits, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read RT Status)\n",
				__func__, rc);
		goto bail_out;
	}

	rc = (bits & (1 << bit)) ? 1 : 0;

bail_out:
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	return rc;
}
EXPORT_SYMBOL(pm8058_irq_get_rt_status);

int pm8058_read(struct pm8058_chip *chip, u16 addr, u8 *values,
		unsigned int len)
{
	if (chip == NULL)
		return -EINVAL;

	return ssbi_read(chip->dev, addr, values, len);
}
EXPORT_SYMBOL(pm8058_read);

int pm8058_write(struct pm8058_chip *chip, u16 addr, u8 *values,
		 unsigned int len)
{
	if (chip == NULL)
		return -EINVAL;

	return ssbi_write(chip->dev, addr, values, len);
}
EXPORT_SYMBOL(pm8058_write);

int pm8058_gpio_config(int gpio, struct pm8058_gpio *param)
{
	return pm8058_gpio_config_h(pmic_chip, gpio, param);
}
EXPORT_SYMBOL(pm8058_gpio_config);

int pm8058_gpio_config_h(struct pm8058_chip *chip, int gpio,
			 struct pm8058_gpio *param)
{
	int	rc;
	u8	bank[8];
	static int	dir_map[] = {
		PM8058_GPIO_MODE_OFF,
		PM8058_GPIO_MODE_OUTPUT,
		PM8058_GPIO_MODE_INPUT,
		PM8058_GPIO_MODE_BOTH,
	};
	unsigned long	irqsave;

	if (param == NULL || chip == NULL)
		return -EINVAL;

	/* Select banks and configure the gpio */
	bank[0] = PM8058_GPIO_WRITE |
		((param->vin_sel << PM8058_GPIO_VIN_SHIFT) &
			PM8058_GPIO_VIN_MASK) |
		PM8058_GPIO_MODE_ENABLE;
	bank[1] = PM8058_GPIO_WRITE |
		((1 << PM8058_GPIO_BANK_SHIFT) &
			PM8058_GPIO_BANK_MASK) |
		((dir_map[param->direction] <<
			PM8058_GPIO_MODE_SHIFT) &
			PM8058_GPIO_MODE_MASK) |
		((param->direction & PM_GPIO_DIR_OUT) ?
			((param->output_buffer & 1) ?
			 PM8058_GPIO_OUT_BUFFER : 0) : 0) |
		((param->direction & PM_GPIO_DIR_OUT) ?
			param->output_value & 0x01 : 0);
	bank[2] = PM8058_GPIO_WRITE |
		((2 << PM8058_GPIO_BANK_SHIFT) &
			PM8058_GPIO_BANK_MASK) |
		((param->pull << PM8058_GPIO_PULL_SHIFT) &
			PM8058_GPIO_PULL_MASK);
	bank[3] = PM8058_GPIO_WRITE |
		((3 << PM8058_GPIO_BANK_SHIFT) &
			PM8058_GPIO_BANK_MASK) |
		((param->out_strength <<
			PM8058_GPIO_OUT_STRENGTH_SHIFT) &
			PM8058_GPIO_OUT_STRENGTH_MASK);
	bank[4] = PM8058_GPIO_WRITE |
		((4 << PM8058_GPIO_BANK_SHIFT) &
			PM8058_GPIO_BANK_MASK) |
		((param->function << PM8058_GPIO_FUNC_SHIFT) &
			PM8058_GPIO_FUNC_MASK);
	bank[5] = PM8058_GPIO_WRITE |
		((5 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		(param->inv_int_pol ? 0 : PM8058_GPIO_NON_INT_POL_INV);

	spin_lock_irqsave(&chip->pm_lock, irqsave);
	/* Remember bank1 for later use */
	chip->gpio_bank1[gpio] = bank[1];
	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_GPIO(gpio), bank, 6);
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	if (rc)
		pr_err("%s: Failed on ssbi_write(): rc=%d (GPIO config)\n",
				__func__, rc);

	return rc;
}
EXPORT_SYMBOL(pm8058_gpio_config_h);

int pm8058_gpio_config_kypd_drv(int gpio_start, int num_gpios)
{
	int	rc;
	struct pm8058_gpio kypd_drv = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_buffer	= PM_GPIO_OUT_BUF_OPEN_DRAIN,
		.output_value	= 0,
		.pull		= PM_GPIO_PULL_NO,
		.vin_sel	= 2,
		.out_strength	= PM_GPIO_STRENGTH_LOW,
		.function	= PM_GPIO_FUNC_1,
		.inv_int_pol	= 1,
	};

	if (gpio_start < 0 || num_gpios < 0 || num_gpios > PM8058_GPIOS)
		return -EINVAL;
	if (pmic_chip == NULL)
		return -ENODEV;

	while (num_gpios--) {
		rc = pm8058_gpio_config_h(pmic_chip, gpio_start++, &kypd_drv);
		if (rc) {
			pr_err("%s: FAIL pm8058_gpio_config(): rc=%d.\n",
				__func__, rc);
			return rc;
		}
	}

	return 0;
}
EXPORT_SYMBOL(pm8058_gpio_config_kypd_drv);

int pm8058_gpio_config_kypd_sns(int gpio_start, int num_gpios)
{
	int	rc;
	struct pm8058_gpio kypd_sns = {
		.direction	= PM_GPIO_DIR_IN,
		.pull		= PM_GPIO_PULL_UP_31P5,
		.vin_sel	= 2,
		.out_strength	= PM_GPIO_STRENGTH_NO,
		.function	= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol	= 1,
	};

	if (gpio_start < 0 || num_gpios < 0 || num_gpios > PM8058_GPIOS)
		return -EINVAL;
	if (pmic_chip == NULL)
		return -ENODEV;

	while (num_gpios--) {
		rc = pm8058_gpio_config_h(pmic_chip, gpio_start++, &kypd_sns);
		if (rc) {
			pr_err("%s: FAIL pm8058_gpio_config(): rc=%d.\n",
				__func__, rc);
			return rc;
		}
	}

	return 0;
}
EXPORT_SYMBOL(pm8058_gpio_config_kypd_sns);

int pm8058_gpio_set_direction(struct pm8058_chip *chip,
			      unsigned gpio, int direction)
{
	int	rc;
	u8	bank1;
	static int	dir_map[] = {
		PM8058_GPIO_MODE_OFF,
		PM8058_GPIO_MODE_OUTPUT,
		PM8058_GPIO_MODE_INPUT,
		PM8058_GPIO_MODE_BOTH,
	};
	unsigned long	irqsave;

	if (!direction || chip == NULL)
		return -EINVAL;

	spin_lock_irqsave(&chip->pm_lock, irqsave);
	bank1 = chip->gpio_bank1[gpio] & ~PM8058_GPIO_MODE_MASK;

	bank1 |= ((dir_map[direction] << PM8058_GPIO_MODE_SHIFT)
		  & PM8058_GPIO_MODE_MASK);

	chip->gpio_bank1[gpio] = bank1;
	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_GPIO(gpio), &bank1, 1);
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	if (rc)
		pr_err("%s: Failed on ssbi_write(): rc=%d (GPIO config)\n",
				__func__, rc);

	return rc;
}

int pm8058_gpio_set(struct pm8058_chip *chip, unsigned gpio, int value)
{
	int	rc;
	u8	bank1;
	unsigned long	irqsave;

	if (gpio >= PM8058_GPIOS || chip == NULL)
		return -EINVAL;

	spin_lock_irqsave(&chip->pm_lock, irqsave);
	bank1 = chip->gpio_bank1[gpio] & ~PM8058_GPIO_OUT_INVERT;

	if (value)
		bank1 |= PM8058_GPIO_OUT_INVERT;

	chip->gpio_bank1[gpio] = bank1;
	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_GPIO(gpio), &bank1, 1);
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	if (rc)
		pr_err("%s: FAIL ssbi_write(): rc=%d. "
		       "(gpio=%d, value=%d)\n",
		       __func__, rc, gpio, value);

	return rc;
}

int pm8058_gpio_get(struct pm8058_chip *chip, unsigned gpio)
{
	int	mode;

	if (gpio >= PM8058_GPIOS || chip == NULL)
		return -EINVAL;

	/* Get gpio value from config bank 1 if output gpio.
	   Get gpio value from IRQ RT status register for all other gpio modes.
	 */
	mode = (chip->gpio_bank1[gpio] & PM8058_GPIO_MODE_MASK) >>
		PM8058_GPIO_MODE_SHIFT;
	if (mode == PM8058_GPIO_MODE_OUTPUT)
		return chip->gpio_bank1[gpio] & PM8058_GPIO_OUT_INVERT;
	else
		return pm8058_irq_get_rt_status(chip,
			PM8058_GPIO_IRQ(chip->pdata.irq_base, gpio));
}

int pm8058_mpp_get(struct pm8058_chip *chip, unsigned mpp)
{
	if (mpp >= PM8058_MPPS || chip == NULL)
		return -EINVAL;

	return pm8058_irq_get_rt_status(chip,
		PM8058_MPP_IRQ(chip->pdata.irq_base, mpp));
}

/* Internal functions */
static inline int
pm8058_config_irq(struct pm8058_chip *chip, u8 *bp, u8 *cp)
{
	int	rc;

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_BLK_SEL, bp, 1);
	if (rc) {
		pr_err("%s: ssbi_write: rc=%d (Select block)\n",
			__func__, rc);
		goto bail_out;
	}

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_CONFIG, cp, 1);
	if (rc)
		pr_err("%s: ssbi_write: rc=%d (Configure IRQ)\n",
			__func__, rc);

bail_out:
	return rc;
}

static void pm8058_irq_mask(unsigned int irq)
{
	int	master, irq_bit;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= chip->pdata.irq_base;
	block = irq / 8;
	master = block / 8;
	irq_bit = irq % 8;

	chip->irqs_allowed[block] &= ~(1 << irq_bit);
	if (!chip->irqs_allowed[block]) {
		chip->blocks_allowed[master] &= ~(1 << (block % 8));

		if (!chip->blocks_allowed[master])
			chip->masters_allowed &= ~(1 << master);
	}

	config = PM8058_IRQF_WRITE | chip->config[irq] |
		PM8058_IRQF_MASK_FE | PM8058_IRQF_MASK_RE;
	pm8058_config_irq(chip, &block, &config);
}

static void pm8058_irq_unmask(unsigned int irq)
{
	int	master, irq_bit;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config, old_irqs_allowed, old_blocks_allowed;

	irq -= chip->pdata.irq_base;
	block = irq / 8;
	master = block / 8;
	irq_bit = irq % 8;

	old_irqs_allowed = chip->irqs_allowed[block];
	chip->irqs_allowed[block] |= 1 << irq_bit;
	if (!old_irqs_allowed) {
		master = block / 8;

		old_blocks_allowed = chip->blocks_allowed[master];
		chip->blocks_allowed[master] |= 1 << (block % 8);

		if (!old_blocks_allowed)
			chip->masters_allowed |= 1 << master;
	}

	config = PM8058_IRQF_WRITE | chip->config[irq];
	pm8058_config_irq(chip, &block, &config);
}

static void pm8058_irq_disable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	pm8058_irq_mask(irq);
	desc->status |= IRQ_MASKED;
}

static void pm8058_irq_enable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	pm8058_irq_unmask(irq);
	desc->status &= ~IRQ_MASKED;
}

static void pm8058_irq_ack(unsigned int irq)
{
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= chip->pdata.irq_base;
	block = irq / 8;

	config = PM8058_IRQF_WRITE | chip->config[irq] | PM8058_IRQF_CLR;
	pm8058_config_irq(chip, &block, &config);
}

static int pm8058_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	int	master, irq_bit;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= chip->pdata.irq_base;
	if (irq > chip->pm_max_irq) {
		chip->pm_max_irq = irq;
		chip->pm_max_blocks =
			chip->pm_max_irq / 8 + 1;
		chip->pm_max_masters =
			chip->pm_max_blocks / 8 + 1;
	}
	block = irq / 8;
	master = block / 8;
	irq_bit = irq % 8;

	chip->config[irq] = (irq_bit << PM8058_IRQF_BITS_SHIFT) |
			PM8058_IRQF_MASK_RE | PM8058_IRQF_MASK_FE;
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		if (flow_type & IRQF_TRIGGER_RISING)
			chip->config[irq] &= ~PM8058_IRQF_MASK_RE;
		if (flow_type & IRQF_TRIGGER_FALLING)
			chip->config[irq] &= ~PM8058_IRQF_MASK_FE;
	} else {
		chip->config[irq] |= PM8058_IRQF_LVL_SEL;

		if (flow_type & IRQF_TRIGGER_HIGH)
			chip->config[irq] &= ~PM8058_IRQF_MASK_RE;
		else
			chip->config[irq] &= ~PM8058_IRQF_MASK_FE;
	}

	config = PM8058_IRQF_WRITE | chip->config[irq] | PM8058_IRQF_CLR;
	return pm8058_config_irq(chip, &block, &config);
}

static int pm8058_irq_set_wake(unsigned int irq, unsigned int on)
{
	struct	pm8058_chip *chip = get_irq_data(irq);

	irq -= chip->pdata.irq_base;
	if (on) {
		if (!chip->wake_enable[irq]) {
			chip->wake_enable[irq] = 1;
			chip->count_wakeable++;
		}
	} else {
		if (chip->wake_enable[irq]) {
			chip->wake_enable[irq] = 0;
			chip->count_wakeable--;
		}
	}

	return 0;
}

static inline int
pm8058_read_root(struct pm8058_chip *chip, u8 *rp)
{
	int	rc;

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_ROOT, rp, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read Root)\n",
			__func__, rc);
		*rp = 0;
	}

	return rc;
}

static inline int
pm8058_read_master(struct pm8058_chip *chip, u8 m, u8 *bp)
{
	int	rc;

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_M_STATUS1 + m, bp, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read Master)\n",
			__func__, rc);
		*bp = 0;
	}

	return rc;
}

static inline int
pm8058_read_block(struct pm8058_chip *chip, u8 *bp, u8 *ip)
{
	int	rc;

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_BLK_SEL, bp, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(): rc=%d (Select Block)\n",
		       __func__, rc);
		*bp = 0;
		goto bail_out;
	}

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_IT_STATUS, ip, 1);
	if (rc)
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read Status)\n",
		       __func__, rc);

bail_out:
	return rc;
}

static irqreturn_t pm8058_isr_thread(int irq_requested, void *data)
{
	struct pm8058_chip *chip = data;
	int	i, j, k;
	u8	root, block, config, bits;
	u8	blocks[MAX_PM_MASTERS];
	int	masters = 0, irq, handled = 0, spurious = 0;
	u16     irqs_to_handle[MAX_PM_IRQ];
	unsigned long	irqsave;

	spin_lock_irqsave(&chip->pm_lock, irqsave);

	/* Read root for masters */
	if (pm8058_read_root(chip, &root))
		goto bail_out;

	masters = root >> 1;

	if (!(masters & chip->masters_allowed) ||
	    (masters & ~chip->masters_allowed)) {
		spurious = 1000000;
	}

	/* Read allowed masters for blocks. */
	for (i = 0; i < chip->pm_max_masters; i++) {
		if (masters & (1 << i)) {
			if (pm8058_read_master(chip, i, &blocks[i]))
				goto bail_out;

			if (!blocks[i]) {
				if (pm8058_can_print())
					pr_err("%s: Spurious master: %d "
					       "(blocks=0)", __func__, i);
				spurious += 10000;
			}
		} else
			blocks[i] = 0;
	}

	/* Select block, read status and call isr */
	for (i = 0; i < chip->pm_max_masters; i++) {
		if (!blocks[i])
			continue;

		for (j = 0; j < 8; j++) {
			if (!(blocks[i] & (1 << j)))
				continue;

			block = i * 8 + j;	/* block # */
			if (pm8058_read_block(chip, &block, &bits))
				goto bail_out;

			if (!bits) {
				if (pm8058_can_print())
					pr_err("%s: Spurious block: "
					       "[master, block]=[%d, %d] "
					       "(bits=0)\n", __func__, i, j);
				spurious += 100;
				continue;
			}

			/* Check IRQ bits */
			for (k = 0; k < 8; k++) {
				if (!(bits & (1 << k)))
					continue;

				/* Check spurious interrupts */
				if (((1 << i) & chip->masters_allowed) &&
				    (blocks[i] & chip->blocks_allowed[i]) &&
				    (bits & chip->irqs_allowed[block])) {

					/* Found one */
					irq = block * 8 + k;
					irqs_to_handle[handled] = irq +
						chip->pdata.irq_base;
					handled++;
				} else {
					/* Clear and mask wrong one */
					config = PM8058_IRQF_W_C_M |
						(k < PM8058_IRQF_BITS_SHIFT);

					pm8058_config_irq(chip,
							  &block, &config);

					if (pm8058_can_print())
						pr_err("%s: Spurious IRQ: "
						       "[master, block, bit]="
						       "[%d, %d (%d), %d]\n",
							__func__,
						       i, j, block, k);
					spurious++;
				}
			}
		}

	}

bail_out:

	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	for (i = 0; i < handled; i++)
		generic_handle_irq(irqs_to_handle[i]);

	if (spurious) {
		if (!pm8058_can_print())
			return IRQ_HANDLED;

		pr_err("%s: spurious = %d (handled = %d)\n",
		       __func__, spurious, handled);
		pr_err("   root = 0x%x (masters_allowed<<1 = 0x%x)\n",
		       root, chip->masters_allowed << 1);
		for (i = 0; i < chip->pm_max_masters; i++) {
			if (masters & (1 << i))
				pr_err("   blocks[%d]=0x%x, "
				       "allowed[%d]=0x%x\n",
				       i, blocks[i],
				       i, chip->blocks_allowed[i]);
		}
	}

	return IRQ_HANDLED;
}

static struct irq_chip pm8058_irq_chip = {
	.name      = "pm8058",
	.ack       = pm8058_irq_ack,
	.mask      = pm8058_irq_mask,
	.unmask    = pm8058_irq_unmask,
	.disable   = pm8058_irq_disable,
	.enable    = pm8058_irq_enable,
	.set_type  = pm8058_irq_set_type,
	.set_wake  = pm8058_irq_set_wake,
};

static int pm8058_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int	i, rc;
	struct	pm8058_platform_data *pdata = client->dev.platform_data;
	struct	pm8058_chip *chip;

	if (pdata == NULL || !client->irq) {
		pr_err("%s: No platform_data or IRQ.\n", __func__);
		return -ENODEV;
	}

	if (pdata->num_subdevs == 0) {
		pr_err("%s: No sub devices to support.\n", __func__);
		return -ENODEV;
	}

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		pr_err("%s: i2c_check_functionality failed.\n", __func__);
		return -ENODEV;
	}

	chip = kzalloc(sizeof *chip, GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	chip->dev = client;

	/* Read PMIC chip revision */
	rc = ssbi_read(chip->dev, SSBI_REG_REV, &chip->revision, 1);
	if (rc)
		pr_err("%s: Failed on ssbi_read for revision: rc=%d.\n",
			__func__, rc);
	pr_info("%s: PMIC revision: %X\n", __func__, chip->revision);

	(void) memcpy((void *)&chip->pdata, (const void *)pdata,
		      sizeof(chip->pdata));

	set_irq_data(chip->dev->irq, (void *)chip);
	set_irq_wake(chip->dev->irq, 1);

	chip->pm_max_irq = 0;
	chip->pm_max_blocks = 0;
	chip->pm_max_masters = 0;

	i2c_set_clientdata(client, chip);

	pmic_chip = chip;
	spin_lock_init(&chip->pm_lock);

	/* Register for all reserved IRQs */
	for (i = pdata->irq_base; i < (pdata->irq_base + MAX_PM_IRQ); i++) {
		set_irq_chip(i, &pm8058_irq_chip);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
		set_irq_data(i, (void *)chip);
	}

	/* Add sub devices with the chip parameter as driver data */
	for (i = 0; i < pdata->num_subdevs; i++)
		pdata->sub_devices[i].driver_data = chip;
	rc = mfd_add_devices(&chip->dev->dev, 0, pdata->sub_devices,
			     pdata->num_subdevs, NULL, 0);

	if (pdata->init) {
		rc = pdata->init(chip);
		if (rc != 0) {
			pr_err("%s: board init failed\n", __func__);
			chip->dev = NULL;
			kfree(chip);
			return -ENODEV;
		}
	}

	rc = request_threaded_irq(chip->dev->irq, NULL, pm8058_isr_thread,
			IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_LOW,
			"pm8058-irq", chip);
	if (rc < 0)
		pr_err("%s: could not request irq %d: %d\n", __func__,
				chip->dev->irq, rc);

	return 0;
}

static int __devexit pm8058_remove(struct i2c_client *client)
{
	struct	pm8058_chip *chip;

	chip = i2c_get_clientdata(client);
	if (chip) {
		if (chip->pm_max_irq) {
			set_irq_wake(chip->dev->irq, 0);
			free_irq(chip->dev->irq, chip);
		}

		chip->dev = NULL;

		kfree(chip);
	}

	return 0;
}

#ifdef CONFIG_PM
static int pm8058_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct	pm8058_chip *chip;
	int	i;
	unsigned long	irqsave;

	chip = i2c_get_clientdata(client);

	for (i = 0; i < MAX_PM_IRQ; i++) {
		spin_lock_irqsave(&chip->pm_lock, irqsave);
		if (chip->config[i] && !chip->wake_enable[i]) {
			if (!((chip->config[i] & PM8058_IRQF_MASK_ALL)
			      == PM8058_IRQF_MASK_ALL))
				pm8058_irq_mask(i + chip->pdata.irq_base);
		}
		spin_unlock_irqrestore(&chip->pm_lock, irqsave);
	}

	if (!chip->count_wakeable)
		disable_irq(chip->dev->irq);

	return 0;
}

static int pm8058_resume(struct i2c_client *client)
{
	struct	pm8058_chip *chip;
	int	i;
	unsigned long	irqsave;

	chip = i2c_get_clientdata(client);

	for (i = 0; i < MAX_PM_IRQ; i++) {
		spin_lock_irqsave(&chip->pm_lock, irqsave);
		if (chip->config[i] && !chip->wake_enable[i]) {
			if (!((chip->config[i] & PM8058_IRQF_MASK_ALL)
			      == PM8058_IRQF_MASK_ALL))
				pm8058_irq_unmask(i + chip->pdata.irq_base);
		}
		spin_unlock_irqrestore(&chip->pm_lock, irqsave);
	}

	if (!chip->count_wakeable)
		enable_irq(chip->dev->irq);

	return 0;
}
#else
#define	pm8058_suspend		NULL
#define	pm8058_resume		NULL
#endif

static const struct i2c_device_id pm8058_ids[] = {
	{ "pm8058-core", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, pm8058_ids);

static struct i2c_driver pm8058_driver = {
	.driver.name	= "pm8058-core",
	.id_table	= pm8058_ids,
	.probe		= pm8058_probe,
	.remove		= __devexit_p(pm8058_remove),
	.suspend	= pm8058_suspend,
	.resume		= pm8058_resume,
};

#if defined(CONFIG_DEBUG_FS)

#define DEBUG_MAX_RW_BUF   128
#define DEBUG_MAX_FNAME    8

static struct dentry *debug_dent;

static char debug_read_buf[DEBUG_MAX_RW_BUF];
static char debug_write_buf[DEBUG_MAX_RW_BUF];

static int debug_gpios[PM8058_GPIOS];

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int debug_read_gpio_bank(int gpio, int bank, u8 *data)
{
	int rc;
	unsigned long irqsave;

	spin_lock_irqsave(&pmic_chip->pm_lock, irqsave);

	*data = bank << PM8058_GPIO_BANK_SHIFT;
	rc = pm8058_write(pmic_chip, SSBI_REG_ADDR_GPIO(gpio), data, 1);
	if (rc)
		goto bail_out;

	*data = bank << PM8058_GPIO_BANK_SHIFT;
	rc = pm8058_read(pmic_chip, SSBI_REG_ADDR_GPIO(gpio), data, 1);

bail_out:
	spin_unlock_irqrestore(&pmic_chip->pm_lock, irqsave);

	return rc;
}

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int gpio = *((int *) file->private_data);
	int len = 0;
	int rc = -EINVAL;
	u8 bank[PM8058_GPIO_BANKS];
	int val = -1;
	int mode;
	int i;

	for (i = 0; i < PM8058_GPIO_BANKS; i++) {
		rc = debug_read_gpio_bank(gpio, i, &bank[i]);
		if (rc)
			pr_err("pmic failed to read bank %d\n", i);
	}

	if (rc) {
		len = snprintf(debug_read_buf, DEBUG_MAX_RW_BUF - 1, "-1\n");
		goto bail_out;
	}

	val = pm8058_gpio_get(pmic_chip, gpio);

	/* print the mode and the value */
	mode = (bank[1] & PM8058_GPIO_MODE_MASK) >>  PM8058_GPIO_MODE_SHIFT;
	if (mode == PM8058_GPIO_MODE_BOTH)
		len = snprintf(debug_read_buf, DEBUG_MAX_RW_BUF - 1,
			       "BOTH %d ", val);
	else if (mode == PM8058_GPIO_MODE_INPUT)
		len = snprintf(debug_read_buf, DEBUG_MAX_RW_BUF - 1,
			       "IN   %d ", val);
	else if (mode == PM8058_GPIO_MODE_OUTPUT)
		len = snprintf(debug_read_buf, DEBUG_MAX_RW_BUF - 1,
			       "OUT  %d ", val);
	else
		len = snprintf(debug_read_buf, DEBUG_MAX_RW_BUF - 1,
			       "OFF  %d ", val);

	/* print the control register values */
	len += snprintf(debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
			bank[0], bank[1], bank[2], bank[3], bank[4], bank[5]);

bail_out:
	rc = simple_read_from_buffer((void __user *) buf, len,
				     ppos, (void *) debug_read_buf, len);

	return rc;
}

static ssize_t debug_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	int gpio = *((int *) file->private_data);
	unsigned long val;
	int mode;

	mode = (pmic_chip->gpio_bank1[gpio] & PM8058_GPIO_MODE_MASK) >>
		PM8058_GPIO_MODE_SHIFT;
	if (mode == PM8058_GPIO_MODE_OFF || mode == PM8058_GPIO_MODE_INPUT)
		return count;

	if (copy_from_user(debug_write_buf, (void __user *) buf, count)) {
		pr_err("failed to copy from user\n");
		return -EFAULT;
	}
	debug_write_buf[count] = '\0';

	(void) strict_strtoul(debug_write_buf, 10, &val);

	if (pm8058_gpio_set(pmic_chip, gpio, val)) {
		pr_err("gpio write failed\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations debug_ops = {
	.open =         debug_open,
	.read =         debug_read,
	.write =        debug_write,
};

static void debug_init(void)
{
	int i;
	char name[DEBUG_MAX_FNAME];

	debug_dent = debugfs_create_dir("pm_gpio", NULL);
	if (IS_ERR(debug_dent)) {
		pr_err("pmic8058 debugfs_create_dir fail, error %ld\n",
		       PTR_ERR(debug_dent));
		return;
	}

	for (i = 0; i < PM8058_GPIOS; i++) {
		snprintf(name, DEBUG_MAX_FNAME-1, "%d", i+1);
		debug_gpios[i] = i;
		if (debugfs_create_file(name, 0644, debug_dent,
					&debug_gpios[i], &debug_ops) == NULL) {
			pr_err("pmic8058 debugfs_create_file %s failed\n",
			       name);
		}
	}
}

static void debug_exit(void)
{
	debugfs_remove_recursive(debug_dent);
}

#else
static void debug_init(void) { }
static void debug_exit(void) { }
#endif

static int __init pm8058_init(void)
{
	int rc = i2c_add_driver(&pm8058_driver);
	pr_notice("%s: i2c_add_driver: rc = %d\n", __func__, rc);

	if (!rc)
		debug_init();

	return rc;
}

static void __exit pm8058_exit(void)
{
	i2c_del_driver(&pm8058_driver);
	debug_exit();
}

arch_initcall(pm8058_init);
module_exit(pm8058_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 core driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-core");
