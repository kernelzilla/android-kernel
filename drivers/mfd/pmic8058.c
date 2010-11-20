/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Qualcomm PMIC8058 driver
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/mfd/pmic8058.h>

/* PMIC8058 Revision */
#define SSBI_REG_REV			0x002  /* PMIC4 revision */

#define PMIC8058_REV_A0			0xE1
#define PMIC8058_REV_B0			0xE2	/* REVISIT */

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

#define PM8058_IRQF_W_C_M		(PM8058_IRQF_WRITE |	\
					PM8058_IRQF_CLR |	\
					PM8058_IRQF_MASK_FE |	\
					PM8058_IRQF_MASK_RE)

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

#define	MAX_PM_IRQ		256
#define	MAX_PM_BLOCKS		(MAX_PM_IRQ / 8 + 1)
#define	MAX_PM_MASTERS		(MAX_PM_BLOCKS / 8 + 1)
#define MAX_PM_GPIO		40

struct pm8058_chip {
	struct pm8058_platform_data	pdata;

	struct completion		irq_completion;

	struct i2c_client		*dev;

	struct task_struct		*pm_task;

	u8	irqs_allowed[MAX_PM_BLOCKS];
	u8	blocks_allowed[MAX_PM_MASTERS];
	u8	masters_allowed;
	u16	irq_i2e[MAX_PM_IRQ];	/* Internal to external mapping */
	int	pm_max_irq;
	int	pm_max_blocks;
	int	pm_max_masters;

	u8	config[PM8058_IRQS];
	u8	revision;
};

static struct pm8058_chip *pmic_chip;

static irqreturn_t pm8058_int_handler(int irq, void *devid);

/* Helper Functions */
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
u8 pmic8058_get_rev(void)
{
	if (pmic_chip == NULL)
		return 0;

	return pmic_chip->revision;
}
EXPORT_SYMBOL(pmic8058_get_rev);

int pmic8058_is_rev_a0(void)
{
	if (pmic_chip == NULL)
		return 0;

	return pmic_chip->revision == PMIC8058_REV_A0;
}
EXPORT_SYMBOL(pmic8058_is_rev_a0);

int pmic8058_is_rev_b0(void)
{
	if (pmic_chip == NULL)
		return 0;

	return pmic_chip->revision == PMIC8058_REV_B0;
}
EXPORT_SYMBOL(pmic8058_is_rev_b0);

int pm8058_read(u16 addr, u8 *values, unsigned int len)
{
	if (pmic_chip == NULL)
		return -ENODEV;

	return ssbi_read(pmic_chip->dev, addr, values, len);
}
EXPORT_SYMBOL(pm8058_read);

int pm8058_write(u16 addr, u8 *values, unsigned int len)
{
	if (pmic_chip == NULL)
		return -ENODEV;

	return ssbi_write(pmic_chip->dev, addr, values, len);
}
EXPORT_SYMBOL(pm8058_write);

int pm8058_gpio_config(int gpio, struct pm8058_gpio *param)
{
	int	rc;
	u8	bank[8];
	static int	dir_map[] = {
		PM8058_GPIO_MODE_OFF,
		PM8058_GPIO_MODE_OUTPUT,
		PM8058_GPIO_MODE_INPUT,
		PM8058_GPIO_MODE_BOTH,
	};

	if (param == NULL)
		return -EINVAL;
	if (pmic_chip == NULL)
		return -ENODEV;

	/* Select banks and configure the gpio */
	bank[0] = PM8058_GPIO_WRITE |
		((param->vin_sel << PM8058_GPIO_VIN_SHIFT) &
			PM8058_GPIO_VIN_MASK) |
		PM8058_GPIO_MODE_ENABLE;
	bank[1] = PM8058_GPIO_WRITE |
		((1 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((dir_map[param->direction] << PM8058_GPIO_MODE_SHIFT) &
			PM8058_GPIO_MODE_MASK) |
		((param->direction & PM_GPIO_DIR_OUT) ?
			PM8058_GPIO_OUT_BUFFER : 0);
	bank[2] = PM8058_GPIO_WRITE |
		((2 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((param->pull << PM8058_GPIO_PULL_SHIFT) &
			PM8058_GPIO_PULL_MASK);
	bank[3] = PM8058_GPIO_WRITE |
		((3 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((param->out_strength << PM8058_GPIO_OUT_STRENGTH_SHIFT) &
			PM8058_GPIO_OUT_STRENGTH_MASK);
	bank[4] = PM8058_GPIO_WRITE |
		((4 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((param->function << PM8058_GPIO_FUNC_SHIFT) &
			PM8058_GPIO_FUNC_MASK);

	rc = ssbi_write(pmic_chip->dev, SSBI_REG_ADDR_GPIO(gpio), bank, 5);
	if (rc) {
		pr_err("%s: Failed on 1st ssbi_write(): rc=%d.\n",
				__func__, rc);
		goto bail_out;
	}

bail_out:
	return rc;
}
EXPORT_SYMBOL(pm8058_gpio_config);

int pm8058_gpio_config_kypd_drv(int gpio_start, int num_gpios)
{
	int	rc;
	struct pm8058_gpio kypd_drv = {
		.direction	= PM_GPIO_DIR_OUT,
		.pull		= PM_GPIO_PULL_NO,
		.vin_sel	= 2,
		.out_strength	= PM_GPIO_STRENGTH_LOW,
		.function	= PM_GPIO_FUNC_1,
		.inv_int_pol	= 1,
	};

	if (gpio_start < 0 || num_gpios < 0 || num_gpios > MAX_PM_GPIO)
		return -EINVAL;

	while (num_gpios--) {
		rc = pm8058_gpio_config(gpio_start++, &kypd_drv);
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
		.pull		= PM_GPIO_PULL_UP1,
		.vin_sel	= 2,
		.out_strength	= PM_GPIO_STRENGTH_NO,
		.function	= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol	= 1,
	};

	if (gpio_start < 0 || num_gpios < 0 || num_gpios > MAX_PM_GPIO)
		return -EINVAL;

	while (num_gpios--) {
		rc = pm8058_gpio_config(gpio_start++, &kypd_sns);
		if (rc) {
			pr_err("%s: FAIL pm8058_gpio_config(): rc=%d.\n",
				__func__, rc);
			return rc;
		}
	}

	return 0;
}
EXPORT_SYMBOL(pm8058_gpio_config_kypd_sns);

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
	int	pm_irq;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= PM8058_FIRST_IRQ;
	pm_irq = chip->pdata.pm_irqs[irq];
	block = pm_irq / 8;
	master = block / 8;
	irq_bit = pm_irq % 8;

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
	int	pm_irq;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config, old_irqs_allowed, old_blocks_allowed;

	irq -= PM8058_FIRST_IRQ;
	pm_irq = chip->pdata.pm_irqs[irq];
	block = pm_irq / 8;
	master = block / 8;
	irq_bit = pm_irq % 8;

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

static void pm8058_irq_ack(unsigned int irq)
{
	int	pm_irq;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= PM8058_FIRST_IRQ;
	pm_irq = chip->pdata.pm_irqs[irq];
	block = pm_irq / 8;

	config = PM8058_IRQF_WRITE | chip->config[irq] | PM8058_IRQF_CLR;
	pm8058_config_irq(chip, &block, &config);
}

static int pm8058_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	int	master, irq_bit;
	int	pm_irq;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= PM8058_FIRST_IRQ;
	pm_irq = chip->pdata.pm_irqs[irq];
	block = pm_irq / 8;
	master = block / 8;
	irq_bit = pm_irq % 8;

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

static void pm8058_handle_isr(struct pm8058_chip *chip)
{
	int	i, j, k;
	u8	root, block, config, bits;
	u8	blocks[MAX_PM_MASTERS];
	int	masters, irq, handled = 0, spurious = 0;

	/* Read root for masters */
	if (pm8058_read_root(chip, &root))
		return;

	masters = root >> 1;

	if (!(masters & chip->masters_allowed) ||
	    (masters & ~chip->masters_allowed)) {
		pr_err("%s: Spurious root: 0x%x (masters=0x%x, "
		       "Allowed masters=0x%x)\n",
		       __func__, root, masters, chip->masters_allowed);
		spurious = 1000000;
	}

	/* Read allowed masters for blocks. */
	for (i = 0; i < chip->pm_max_masters; i++) {
		if (masters & (1 << i)) {
			if (pm8058_read_master(chip, i, &blocks[i]))
				goto bail_out;

			if (!blocks[i]) {
				pr_err("%s: Spurious master: %d (blocks=0)",
					__func__, i);
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
				pr_err("%s: Spurious block: "
				       "[master, block] ="
				       "[%d, %d] (bits=0)\n",
					__func__, i, j);
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
					irq = chip->irq_i2e[irq];
					generic_handle_irq(irq);
					handled++;
				} else {
					/* Clear and mask wrong one */
					config = PM8058_IRQF_W_C_M |
						(k < PM8058_IRQF_BITS_SHIFT);

					pm8058_config_irq(chip,
							  &block, &config);

					pr_err("%s: Spurious IRQ: "
					       "[master, block, bit]="
					       "[%d, %d (%d), %d]\n",
						__func__, i, j, block, k);
					pr_err("%s: Allowed "
					       "[masters, blocks, irqs]="
					       "[0x%x, 0x%x, 0x%x]\n",
					       __func__,
					       chip->masters_allowed,
					       chip->blocks_allowed[i],
					       chip->irqs_allowed[block]);
					spurious++;
				}
			}
		}

	}

bail_out:
	if (spurious) {
		pr_err("%s: handled = %d, spurious = %d\n",
		       __func__, handled, spurious);
		pr_err("   root=0x%x, masters_allowed=0x%x\n",
		       root, chip->masters_allowed);
		for (i = 0; i < chip->pm_max_masters; i++) {
			if (masters & (1 << i))
				pr_err("   blocks[%d]=0x%x, "
				       "allowed[%d]=0x%x\n",
				       i, blocks[i],
				       i, chip->blocks_allowed[i]);
		}
	}

	return;
}

static int pm8058_ist(void *data)
{
	unsigned int irq = (unsigned int)data;
	struct pm8058_chip *chip = get_irq_data(irq);

	if (!chip) {
		pr_err("%s: Invalid chip data: IRQ=%d\n", __func__, irq);
		return -EINVAL;
	}

	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		wait_for_completion_interruptible(&chip->irq_completion);

		local_irq_disable();
		pm8058_handle_isr(chip);
		local_irq_enable();

		enable_irq(irq);
	}

	return 0;
}

static struct task_struct *pm8058_init_ist(unsigned int irq,
					   struct completion *irq_completion)
{
	struct task_struct *thread;

	init_completion(irq_completion);
	thread = kthread_run(pm8058_ist, (void *)irq, "pm8058-ist");
	if (!thread)
		pr_err("%s: Failed to create kernel thread for (irq=%d)\n",
			__func__, irq);

	return thread;
}

static irqreturn_t pm8058_int_handler(int irq, void *devid)
{
	disable_irq(irq);
	complete(devid);

	return IRQ_HANDLED;
}

static struct irq_chip pm8058_irq_chip = {
	.name      = "pm8058",
	.ack       = pm8058_irq_ack,
	.mask      = pm8058_irq_mask,
	.unmask    = pm8058_irq_unmask,
	.set_type  = pm8058_irq_set_type,
};

static int pm8058_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int	i, pm_irq, rc;
	struct	pm8058_platform_data *pdata = client->dev.platform_data;
	struct	pm8058_chip *chip;

	if (pdata == NULL || !client->irq) {
		pr_err("%s: No platform_data or IRQ.\n",
			__func__);
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

	chip->pm_task = pm8058_init_ist(chip->dev->irq, &chip->irq_completion);
	if (chip->pm_task == NULL) {
		pr_err("%s: pm8058_init_ist() failed\n", __func__);
		kfree(chip);
		return -ESRCH;
	}

	chip->pm_max_irq = 0;
	for (i = 0; i < PM8058_IRQS; i++) {
		pm_irq = chip->pdata.pm_irqs[i];
		if (pm_irq > 0)
			chip->irq_i2e[pm_irq] = i + PM8058_FIRST_IRQ;
		if (pm_irq > chip->pm_max_irq)
			chip->pm_max_irq = pm_irq;
	}

	i2c_set_clientdata(client, chip);

	pmic_chip = chip;

	if (chip->pm_max_irq <= 0) {
		pr_err("%s: No pm_irq[] table.\n", __func__);
		/* This is the case of no interrupt. */
		return 0;
	}

	chip->pm_max_blocks = chip->pm_max_irq / 8 + 1;
	chip->pm_max_masters = chip->pm_max_blocks / 8 + 1;

	/* Register for all reserved IRQs */
	for (i = PM8058_FIRST_IRQ; i < (PM8058_FIRST_IRQ + PM8058_IRQS); i++) {
		set_irq_chip(i, &pm8058_irq_chip);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
		set_irq_data(i, (void *)chip);
	}

	rc = request_irq(chip->dev->irq, pm8058_int_handler,
				IRQF_DISABLED | IRQF_TRIGGER_LOW,
				"pm8058-irq", &chip->irq_completion);
	if (rc < 0)
		pr_err("%s: could not request irq %d: %d\n", __func__,
					 chip->dev->irq, rc);
	else {
		set_irq_data(chip->dev->irq, (void *)chip);
		set_irq_wake(chip->dev->irq, 1);
	}

	return 0;
}

static int __devexit pm8058_remove(struct i2c_client *client)
{
	struct	pm8058_chip *chip;

	chip = i2c_get_clientdata(client);
	if (chip) {
		if (chip->pm_max_irq) {
			set_irq_wake(chip->dev->irq, 0);
			free_irq(chip->dev->irq, &chip->irq_completion);
		}

		if (chip->pm_task != NULL)
			kthread_stop(chip->pm_task);

		chip->dev = NULL;

		kfree(chip);
	}

	return 0;
}

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
};

static int __init pm8058_init(void)
{
	int rc = i2c_add_driver(&pm8058_driver);
	pr_notice("%s: i2c_add_driver: rc = %d\n", __func__, rc);
	return rc;
}

static void __exit pm8058_exit(void)
{
	i2c_del_driver(&pm8058_driver);
}

subsys_initcall(pm8058_init);
module_exit(pm8058_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("PMIC8058 core driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-core");
