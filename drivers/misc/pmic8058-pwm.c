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
 * Qualcomm PMIC8058 PWM driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-pwm.h>

#define	PM8058_LPG_BANKS		8
#define	PM8058_PWM_CHANNELS		PM8058_LPG_BANKS	/* MAX=8 */

#define	PM8058_LPG_CTL_REGS		7

/* PMIC8058 LPG/PWM */
#define	SSBI_REG_ADDR_LPG_CTL_BASE	0x13C
#define	SSBI_REG_ADDR_LPG_CTL(n)	(SSBI_REG_ADDR_LPG_CTL_BASE + (n))
#define	SSBI_REG_ADDR_LPG_BANK_SEL	0x143
#define	SSBI_REG_ADDR_LPG_BANK_EN	0x144
#define	SSBI_REG_ADDR_LPG_LUT_CFG0	0x145
#define	SSBI_REG_ADDR_LPG_LUT_CFG1	0x146
#define	SSBI_REG_ADDR_LPG_TEST		0x147

/* Control 0 */
#define	PM8058_PWM_1KHZ_COUNT_MASK	0xF0
#define	PM8058_PWM_1KHZ_COUNT_SHIFT	4

#define	PM8058_PWM_OUTPUT_EN		0x08
#define	PM8058_PWM_PWM_EN		0x04
#define	PM8058_PWM_RAMP_GEN_EN		0x02
#define	PM8058_PWM_RAMP_START		0x01

/* Control 1 */
#define	PM8058_PWM_TOGGLE_EN		0x80
#define	PM8058_PWM_BYPASS_LUT		0x40
#define	PM8058_PWM_HIGH_INDEX_MASK	0x3F

/* Control 2 */
#define	PM8058_PWM_LOOP_EN		0x80
#define	PM8058_PWM_RAMP_UP		0x40
#define	PM8058_PWM_LOW_INDEX_MASK	0x3F

/* Control 3 */
#define	PM8058_PWM_VALUE_BIT7_0		0xFF
#define	PM8058_PWM_VALUE_BIT5_0		0x3F

/* Control 4 */
#define	PM8058_PWM_VALUE_BIT8		0x80

#define	PM8058_PWM_CLK_SEL_MASK		0x60
#define	PM8058_PWM_CLK_SEL_SHIFT	5

#define	PM8058_PWM_CLK_SEL_NO		0
#define	PM8058_PWM_CLK_SEL_1KHZ		1
#define	PM8058_PWM_CLK_SEL_32KHZ	2
#define	PM8058_PWM_CLK_SEL_19P2MHZ	3

#define	PM8058_PWM_PREDIVIDE_MASK	0x18
#define	PM8058_PWM_PREDIVIDE_SHIFT	3

#define	PM8058_PWM_PREDIVIDE_2		0
#define	PM8058_PWM_PREDIVIDE_3		1
#define	PM8058_PWM_PREDIVIDE_5		2
#define	PM8058_PWM_PREDIVIDE_6		3

#define	PM8058_PWM_M_MASK	0x07
#define	PM8058_PWM_M_MIN	0
#define	PM8058_PWM_M_MAX	7

/* Control 5 */
#define	PM8058_PWM_PAUSE_COUNT_HIGH_MASK	0xFC
#define	PM8058_PWM_PAUSE_COUNT_HIGH_SHIFT	6

#define	PM8058_PWM_PAUSE_ENABLE_HIGH		0x02
#define	PM8058_PWM_SIZE_9_BIT			0x01

/* Control 6 */
#define	PM8058_PWM_PAUSE_COUNT_LOW_MASK		0xFC
#define	PM8058_PWM_PAUSE_COUNT_LOW_SHIFT	6

#define	PM8058_PWM_PAUSE_ENABLE_LOW		0x02
#define	PM8058_PWM_RESERVED			0x01

/* TEST */
#define	PM8058_PWM_DTEST_MASK		0x38
#define	PM8058_PWM_DTEST_SHIFT		3

#define	PM8058_PWM_DTEST_BANK_MASK	0x07

/* PWM frequency support
 *
 * PWM Frequency = Clock Frequency / (N * T)
 * 	or
 * PWM Period = Clock Period * (N * T)
 * 	where
 * N = 2^9 or 2^6 for 9-bit or 6-bit PWM size
 * T = Pre-divide * 2^m, m = 0..7 (exponent)
 *
 * We use this formula to figure out m for the best pre-divide and clock:
 * (PWM Period / N) / 2^m = (Pre-divide * Clock Period)
*/
#define	NUM_CLOCKS	3

#define	NSEC_1000HZ	(NSEC_PER_SEC / 1000)
#define	NSEC_32768KHZ	(NSEC_PER_SEC / 32768)
#define	NSEC_19P2MHZ	(NSEC_PER_SEC / 19200000)

#define	CLK_PERIOD_MIN	NSEC_19P2MHZ
#define	CLK_PERIOD_MAX	NSEC_1000HZ

#define	NUM_PRE_DIVIDE	3	/* No default support for pre-divide = 6 */

#define	PRE_DIVIDE_0		2
#define	PRE_DIVIDE_1		3
#define	PRE_DIVIDE_2		5

#define	PRE_DIVIDE_MIN		PRE_DIVIDE_0
#define	PRE_DIVIDE_MAX		PRE_DIVIDE_2

static unsigned long pt_t[NUM_PRE_DIVIDE][NUM_CLOCKS] = {
	{	PRE_DIVIDE_0 * NSEC_1000HZ,
		PRE_DIVIDE_0 * NSEC_32768KHZ,
		PRE_DIVIDE_0 * NSEC_19P2MHZ,
	},
	{	PRE_DIVIDE_1 * NSEC_1000HZ,
		PRE_DIVIDE_1 * NSEC_32768KHZ,
		PRE_DIVIDE_1 * NSEC_19P2MHZ,
	},
	{	PRE_DIVIDE_2 * NSEC_1000HZ,
		PRE_DIVIDE_2 * NSEC_32768KHZ,
		PRE_DIVIDE_2 * NSEC_19P2MHZ,
	},
};

#define	MIN_MPT	((1 << PM8058_PWM_M_MIN) * PRE_DIVIDE_MIN * CLK_PERIOD_MIN)
#define	MAX_MPT	((1 << PM8058_PWM_M_MAX) * PRE_DIVIDE_MAX * CLK_PERIOD_MAX)

/* Private data */
struct pm8058_pwm_chip;

struct pwm_device {
	int			pwm_id;		/* = bank/channel id */
	int			in_use;
	const char		*label;
	int			pwm_period;
	int			pwm_duty;
	u8			pwm_ctl[PM8058_LPG_CTL_REGS];
	int			irq;
	struct pm8058_pwm_chip	*chip;
};

struct pm8058_pwm_chip {
	struct pwm_device	pwm_dev[PM8058_PWM_CHANNELS];
	u8			bank_mask;
	struct mutex		pwm_mutex;
	struct pm8058_chip	*pm_chip;
};

static struct pm8058_pwm_chip	*pwm_chip;

/* Internal functions */
static inline int pm8058_pwm_bank_enable(struct pwm_device *pwm, int enable)
{
	int	rc;
	u8	reg;
	struct pm8058_pwm_chip	*chip;

	chip = pwm->chip;

	if (enable)
		reg = chip->bank_mask | (1 << pwm->pwm_id);
	else
		reg = chip->bank_mask & ~(1 << pwm->pwm_id);

	rc = pm8058_write(chip->pm_chip, SSBI_REG_ADDR_LPG_BANK_EN, &reg, 1);
	if (rc) {
		pr_err("%s: pm8058_write(): rc=%d (Enable LPG Bank)\n",
		       __func__, rc);
		goto bail_out;
	}
	chip->bank_mask = reg;

bail_out:
	return rc;
}

static inline int pm8058_pwm_bank_sel(struct pwm_device *pwm)
{
	int	rc;
	u8	reg;

	reg = pwm->pwm_id;
	rc = pm8058_write(pwm->chip->pm_chip, SSBI_REG_ADDR_LPG_BANK_SEL,
			     &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write(): rc=%d (Select PWM Bank)\n",
		       __func__, rc);
	return rc;
}

static inline int pm8058_pwm_enable(struct pwm_device *pwm, int enable)
{
	int	rc;
	u8	reg;

	if (enable)
		reg = pwm->pwm_ctl[0] | PM8058_PWM_PWM_EN;
	else
		reg = pwm->pwm_ctl[0] & ~PM8058_PWM_PWM_EN;

	rc = pm8058_write(pwm->chip->pm_chip, SSBI_REG_ADDR_LPG_CTL(0),
			  &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write(): rc=%d (Enable PWM Ctl 0)\n",
		       __func__, rc);
	else
		pwm->pwm_ctl[0] = reg;
	return rc;
}

static inline int pm8058_pwm_output_enable(struct pwm_device *pwm, int enable)
{
	int	rc;
	u8	reg;

	if (enable)
		reg = pwm->pwm_ctl[0] | PM8058_PWM_OUTPUT_EN;
	else
		reg = pwm->pwm_ctl[0] & ~PM8058_PWM_OUTPUT_EN;

	rc = pm8058_write(pwm->chip->pm_chip, SSBI_REG_ADDR_LPG_CTL(0),
			  &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write(): rc=%d (Enable PWM Ctl 0)\n",
		       __func__, rc);
	else
		pwm->pwm_ctl[0] = reg;
	return rc;
}

static inline int pm8058_pwm_ramp_generator(struct pwm_device *pwm, int enable)
{
	int	rc;
	u8	reg;

	if (enable)
		reg = pwm->pwm_ctl[0] | PM8058_PWM_RAMP_GEN_EN;
	else
		reg = pwm->pwm_ctl[0] & ~PM8058_PWM_RAMP_GEN_EN;

	rc = pm8058_write(pwm->chip->pm_chip, SSBI_REG_ADDR_LPG_CTL(0),
			  &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write(): rc=%d (Enable PWM Ctl 0)\n",
		       __func__, rc);
	else
		pwm->pwm_ctl[0] = reg;
	return rc;
}

static inline int pm8058_pwm_ramp_start(struct pwm_device *pwm)
{
	int	rc;
	u8	reg;

	reg = pwm->pwm_ctl[0] | PM8058_PWM_RAMP_START;
	rc = pm8058_write(pwm->chip->pm_chip, SSBI_REG_ADDR_LPG_CTL(0),
			  &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write(): rc=%d (Enable PWM Ctl 0)\n",
		       __func__, rc);
	return rc;
}

static void pm8058_pwm_start(struct pwm_device *pwm, int start)
{
	pm8058_pwm_bank_sel(pwm);
	if (start) {
		pm8058_pwm_output_enable(pwm, 1);
		pm8058_pwm_enable(pwm, 1);
	} else {
		pm8058_pwm_output_enable(pwm, 0);
		pm8058_pwm_enable(pwm, 0);
	}
}

static int pm8058_pwm_configure(struct pwm_device *pwm,
			 struct pw8058_pwm_config *pwm_conf)
{
	int	i, rc;
	u8	reg;

	pm8058_pwm_bank_sel(pwm);

	reg = (pwm_conf->pwm_size > 6) ? PM8058_PWM_SIZE_9_BIT : 0;
	pwm->pwm_ctl[5] = reg;

	reg = (pwm_conf->clk << PM8058_PWM_CLK_SEL_SHIFT)
		& PM8058_PWM_CLK_SEL_MASK;
	reg |= (pwm_conf->pre_div << PM8058_PWM_PREDIVIDE_SHIFT)
		& PM8058_PWM_PREDIVIDE_MASK;
	reg |= pwm_conf->pre_div_exp & PM8058_PWM_M_MASK;
	pwm->pwm_ctl[4] = reg;

	if (pwm_conf->pwm_size > 6) {
		pwm->pwm_ctl[3] = pwm_conf->pwm_value
					& PM8058_PWM_VALUE_BIT7_0;
		pwm->pwm_ctl[4] |= (pwm_conf->pwm_value >> 1)
					& PM8058_PWM_VALUE_BIT8;
	} else {
		pwm->pwm_ctl[3] = pwm_conf->pwm_value
					& PM8058_PWM_VALUE_BIT5_0;
	}

	if (pwm_conf->bypass_lut) {
		pwm->pwm_ctl[2] = 0;
		pwm->pwm_ctl[1] = PM8058_PWM_BYPASS_LUT;
	} else {
		pwm->pwm_ctl[2] = 0;
		pwm->pwm_ctl[1] = 0;
	}

	for (i = 0; i < 5; i++) {
		rc = pm8058_write(pwm->chip->pm_chip,
				  SSBI_REG_ADDR_LPG_CTL(i+1),
				  &pwm->pwm_ctl[i+1], 1);
		if (rc) {
			pr_err("%s: pm8058_write(): rc=%d (PWM Ctl[%d])\n",
			       __func__, rc, i+1);
			break;
		}
	}

	return rc;
}

/* APIs */
/*
 * pwm_request - request a PWM device
 */
struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device	*pwm;

	if (pwm_id > PM8058_PWM_CHANNELS || pwm_id < 0)
		return ERR_PTR(-EINVAL);
	if (pwm_chip == NULL)
		return ERR_PTR(-ENODEV);

	mutex_lock(&pwm_chip->pwm_mutex);
	pwm = &pwm_chip->pwm_dev[pwm_id];
	if (!pwm->in_use) {
		pwm->in_use = 1;
		pwm->label = label;
	} else
		pwm = ERR_PTR(-EBUSY);
	mutex_unlock(&pwm_chip->pwm_mutex);

	return pwm;
}
EXPORT_SYMBOL(pwm_request);

/*
 * pwm_free - free a PWM device
 */
void pwm_free(struct pwm_device *pwm)
{
	if (pwm == NULL || IS_ERR(pwm) || pwm->chip == NULL)
		return;

	mutex_lock(&pwm->chip->pwm_mutex);
	if (pwm->in_use) {
		pwm->in_use = 0;
		pwm->label = NULL;
	}
	mutex_unlock(&pwm->chip->pwm_mutex);
}
EXPORT_SYMBOL(pwm_free);

/*
 * pwm_config - change a PWM device configuration
 */
int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	int	n, m, clk, div;
	int	best_m, best_div, best_clk;
	int	last_err, cur_err, better_err, better_m;
	unsigned int	tmp_p, last_p, min_err, period;
	struct pw8058_pwm_config	pwm_conf;
	int	rc;

	if (pwm == NULL || IS_ERR(pwm) ||
		(unsigned)duty_ns > (unsigned)period_ns)
		return -EINVAL;
	if (pwm->chip == NULL)
		return -ENODEV;

	mutex_lock(&pwm->chip->pwm_mutex);

	if (!pwm->in_use) {
		rc = -EINVAL;
		goto out_unlock;
	}

	/* PWM Period / N */
	period = (unsigned int)period_ns >> 6;
	if (period >= MAX_MPT) {
		n = 9;
		period >>= 3;
	} else
		n = 6;

	min_err = MAX_MPT;
	best_m = 0;
	best_clk = 0;
	best_div = 0;
	for (clk = 0; clk < NUM_CLOCKS; clk++) {
		tmp_p = period;
		last_p = tmp_p;
		for (div = 0; div < NUM_PRE_DIVIDE; div++) {
			for (m = 0; m < PM8058_PWM_M_MAX; m++) {
				if (tmp_p <= pt_t[div][clk]) {
					/* Found local best */
					if (!m) {
						better_err = pt_t[div][clk] -
							tmp_p;
						better_m = m;
					} else {
						last_err = last_p -
							pt_t[div][clk];
						cur_err = pt_t[div][clk] -
							tmp_p;

						if (cur_err < last_err) {
							better_err = cur_err;
							better_m = m;
						} else {
							better_err = last_err;
							better_m = m - 1;
						}
					}

					if (better_err < min_err) {
						min_err = better_err;
						best_m = better_m;
						best_clk = clk;
						best_div = div;
					}
					break;
				} else {
					last_p = tmp_p;
					tmp_p >>= 1;
				}

			}
		}
	}

	pwm_conf.pwm_size = n;
	pwm_conf.bypass_lut = 1;
	pwm_conf.clk = best_clk;
	pwm_conf.pre_div = best_div;
	pwm_conf.pre_div_exp = best_m;
	/* Subtract by 1 to avoid overflow when duty_ns==period_ns */
	if (duty_ns)
		duty_ns--;
	pwm_conf.pwm_value = duty_ns / period;

	rc = pm8058_pwm_configure(pwm, &pwm_conf);

	pr_debug("%s: min_err=%d (pwm_p=%u) [m=%d, clk=%d, div=%d]", __func__,
	       min_err, (unsigned)period_ns, best_m, best_clk, best_div);

out_unlock:
	mutex_unlock(&pwm->chip->pwm_mutex);
	return rc;
}
EXPORT_SYMBOL(pwm_config);

/*
 * pwm_configure - change a PWM device configuration
 */
int pwm_configure(struct pwm_device *pwm, struct pw8058_pwm_config *pwm_conf)
{
	int	rc;

	if (pwm == NULL || IS_ERR(pwm) || pwm_conf == NULL)
		return -EINVAL;
	if (pwm->chip == NULL)
		return -ENODEV;

	mutex_lock(&pwm->chip->pwm_mutex);
	if (!pwm->in_use)
		rc = -EINVAL;
	else
		rc = pm8058_pwm_configure(pwm, pwm_conf);
	mutex_unlock(&pwm->chip->pwm_mutex);
	return rc;
}
EXPORT_SYMBOL(pwm_configure);

/*
 * pwm_enable - start a PWM output toggling
 */
int pwm_enable(struct pwm_device *pwm)
{
	int	rc;

	if (pwm == NULL || IS_ERR(pwm))
		return -EINVAL;
	if (pwm->chip == NULL)
		return -ENODEV;

	mutex_lock(&pwm->chip->pwm_mutex);
	if (!pwm->in_use)
		rc = -EINVAL;
	else {
		rc = pm8058_pwm_bank_enable(pwm, 1);
		pm8058_pwm_start(pwm, 1);
	}
	mutex_unlock(&pwm->chip->pwm_mutex);
	return rc;
}
EXPORT_SYMBOL(pwm_enable);

/*
 * pwm_disable - stop a PWM output toggling
 */
void pwm_disable(struct pwm_device *pwm)
{
	if (pwm == NULL || IS_ERR(pwm) || pwm->chip == NULL)
		return;

	mutex_lock(&pwm->chip->pwm_mutex);
	if (pwm->in_use) {
		pm8058_pwm_start(pwm, 0);
		pm8058_pwm_bank_enable(pwm, 0);
	}
	mutex_unlock(&pwm->chip->pwm_mutex);
}
EXPORT_SYMBOL(pwm_disable);

int pwm_set_dtest(struct pwm_device *pwm, int enable)
{
	int	rc;
	u8	reg;

	if (pwm == NULL || IS_ERR(pwm))
		return -EINVAL;
	if (pwm->chip == NULL)
		return -ENODEV;

	mutex_lock(&pwm->chip->pwm_mutex);
	if (!pwm->in_use)
		rc = -EINVAL;
	else {
		reg = pwm->pwm_id & PM8058_PWM_DTEST_BANK_MASK;
		if (enable)
			/* Only Test 1 available */
			reg |= (1 << PM8058_PWM_DTEST_SHIFT) &
				PM8058_PWM_DTEST_MASK;
		rc = pm8058_write(pwm->chip->pm_chip, SSBI_REG_ADDR_LPG_TEST,
				  &reg, 1);
		if (rc)
			pr_err("%s: pm8058_write(DTEST=0x%x): rc=%d\n",
			       __func__, reg, rc);

	}
	mutex_unlock(&pwm->chip->pwm_mutex);
	return rc;
}
EXPORT_SYMBOL(pwm_set_dtest);

static int __devinit pmic8058_pwm_probe(struct platform_device *pdev)
{
	struct pm8058_chip	*pm_chip;
	struct pm8058_pwm_chip	*chip;
	int	i;

	pm_chip = platform_get_drvdata(pdev);
	if (pm_chip == NULL) {
		pr_err("%s: no parent data passed in.\n", __func__);
		return -EFAULT;
	}

	chip = kzalloc(sizeof *chip, GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < PM8058_PWM_CHANNELS; i++) {
		chip->pwm_dev[i].pwm_id = i;
		chip->pwm_dev[i].chip = chip;
	}

	mutex_init(&chip->pwm_mutex);

	chip->pm_chip = pm_chip;
	pwm_chip = chip;
	platform_set_drvdata(pdev, chip);

	pr_notice("%s: OK\n", __func__);
	return 0;
}

static int __devexit pmic8058_pwm_remove(struct platform_device *pdev)
{
	struct pm8058_pwm_chip	*chip = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	kfree(chip);
	return 0;
}

static struct platform_driver pmic8058_pwm_driver = {
	.probe		= pmic8058_pwm_probe,
	.remove		= __devexit_p(pmic8058_pwm_remove),
	.driver		= {
		.name = "pm8058-pwm",
		.owner = THIS_MODULE,
	},
};

static int __init pm8058_pwm_init(void)
{
	return platform_driver_register(&pmic8058_pwm_driver);
}

static void __exit pm8058_pwm_exit(void)
{
	platform_driver_unregister(&pmic8058_pwm_driver);
}

subsys_initcall(pm8058_pwm_init);
module_exit(pm8058_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 PWM driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058_pwm");
