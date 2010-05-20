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
 */

#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/pmic8058-regulator.h>

/* Regulator types */
#define REGULATOR_TYPE_LDO		0
#define REGULATOR_TYPE_SMPS		1
#define REGULATOR_TYPE_LVS		2

/* Common masks */
#define REGULATOR_EN_MASK		0x80
#define REGULATOR_BANK_SEL(n)		((n) << 4)
#define REGULATOR_BANK_WRITE		0x80

/* LDO programming */
#define LDO_CTRL_VPROG_MASK		0x1F

#define LDO_TEST_VREF_UPDATE_MASK	0x08
#define LDO_TEST_VREF_MASK		0x04
#define LDO_TEST_FINE_STEP_MASK		0x02
#define LDO_TEST_RANGE_EXTN_MASK	0x01

#define PLDO_LOW_UV_MIN			750000
#define PLDO_LOW_UV_MAX			1525000
#define PLDO_LOW_UV_STEP		25000
#define PLDO_LOW_FINE_STEP_UV		12500

#define PLDO_NORM_UV_MIN		1500000
#define PLDO_NORM_UV_MAX		3050000
#define PLDO_NORM_UV_STEP		50000
#define PLDO_NORM_FINE_STEP_UV		25000

#define PLDO_HIGH_UV_MIN		1750000
#define PLDO_HIGH_UV_MAX		3250000
#define PLDO_HIGH_UV_STEP		100000
#define PLDO_HIGH_FINE_STEP_UV		50000

#define NLDO_UV_MIN			750000
#define NLDO_UV_MAX			1525000
#define NLDO_UV_STEP			25000
#define NLDO_FINE_STEP_UV		12500

/* SMPS masks and values */
#define SMPS_VREF_SEL_MASK		0x20
#define SMPS_VPROG_MASK			0x1F
#define SMPS_VLOW_SEL_MASK		0x01

#define SMPS_MODE1_UV_MIN		1500000
#define SMPS_MODE1_UV_MAX		3050000
#define SMPS_MODE1_STEP			50000

#define SMPS_MODE2_UV_MIN		750000
#define SMPS_MODE2_UV_MAX		1525000
#define SMPS_MODE2_STEP			25000

#define SMPS_MODE3_UV_MIN		375000
#define SMPS_MODE3_UV_MAX		1150000
#define SMPS_MODE3_STEP			25000

#define SMPS_UV_MIN			SMPS_MODE3_UV_MIN
#define SMPS_UV_MAX			SMPS_MODE1_UV_MAX

#define SMPS_IS_MODE3(uV)		((uV) < SMPS_MODE2_UV_MIN)
#define SMPS_IS_MODE2(uV)		((uV) < SMPS_MODE1_UV_MIN)

struct pm8058_vreg {
	const char			*name;
	u16				ctrl_addr;
	u16				test_addr;
	u16				test2_addr;
	u8				type;
	u8				ctrl_reg;
	u8				test_reg;
	u8				test_reg2;
	u8				is_nmos;
	struct regulator_consumer_supply rsupply;
	struct regulator_desc		rdesc;
	struct regulator_init_data	rdata;
	struct regulator_dev		*rdev;
	struct platform_device		*pdev;
};

#define LDO(_name, _ctrl_addr, _test_addr, _is_nmos) { \
	.name = _name, \
	.ctrl_addr = _ctrl_addr, \
	.test_addr = _test_addr, \
	.type = REGULATOR_TYPE_LDO, \
	.rsupply = { \
		.supply = _name, \
	}, \
	.rdata = { \
		.constraints = { \
			.name = _name, \
			.valid_modes_mask = REGULATOR_MODE_NORMAL, \
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | \
				REGULATOR_CHANGE_STATUS, \
		}, \
		.num_consumer_supplies = 1, \
	}, \
}

#define SMPS(_name, _ctrl_addr, _test2_addr) \
{ \
	.name = _name, \
	.ctrl_addr = _ctrl_addr, \
	.test2_addr = _test2_addr, \
	.type = REGULATOR_TYPE_SMPS, \
	.rsupply = { \
		.supply = _name, \
	}, \
	.rdata = { \
		.constraints = { \
			.name = _name, \
			.valid_modes_mask = REGULATOR_MODE_NORMAL, \
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | \
				REGULATOR_CHANGE_STATUS, \
		}, \
		.num_consumer_supplies = 1, \
	}, \
}

#define LVS(_name, _ctrl_addr) \
{ \
	.name = _name, \
	.ctrl_addr = _ctrl_addr, \
	.type = REGULATOR_TYPE_LVS, \
	.rsupply = { \
		.supply = _name, \
	}, \
	.rdata = { \
		.constraints = { \
			.name = _name, \
			.valid_modes_mask = REGULATOR_MODE_NORMAL, \
			.valid_ops_mask = REGULATOR_CHANGE_STATUS, \
		}, \
		.num_consumer_supplies = 1, \
	}, \
}

static struct pm8058_vreg pm8058_vreg[] = {
	/*   name       ctrl   test   n/p */
	LDO("8058_l0",  0x009, 0x065, 1),
	LDO("8058_l1",  0x00A, 0x066, 1),
	LDO("8058_l2",  0x00B, 0x067, 0),
	LDO("8058_l3",  0x00C, 0x068, 0),
	LDO("8058_l4",  0x00D, 0x069, 0),
	LDO("8058_l5",  0x00E, 0x06A, 0),
	LDO("8058_l6",  0x00F, 0x06B, 0),
	LDO("8058_l7",  0x010, 0x06C, 0),
	LDO("8058_l8",  0x011, 0x06D, 0),
	LDO("8058_l9",  0x012, 0x06E, 0),
	LDO("8058_l10", 0x013, 0x06F, 0),
	LDO("8058_l11", 0x014, 0x070, 0),
	LDO("8058_l12", 0x015, 0x071, 0),
	LDO("8058_l13", 0x016, 0x072, 0),
	LDO("8058_l14", 0x017, 0x073, 0),
	LDO("8058_l15", 0x089, 0x0E5, 0),
	LDO("8058_l16", 0x08A, 0x0E6, 0),
	LDO("8058_l17", 0x08B, 0x0E7, 0),
	LDO("8058_l18", 0x11D, 0x125, 0),
	LDO("8058_l19", 0x11E, 0x126, 0),
	LDO("8058_l20", 0x11F, 0x127, 0),
	LDO("8058_l21", 0x120, 0x128, 1),
	LDO("8058_l22", 0x121, 0x129, 1),
	LDO("8058_l23", 0x122, 0x12A, 1),
	LDO("8058_l24", 0x123, 0x12B, 1),
	LDO("8058_l25", 0x124, 0x12C, 1),

	/*    name      ctrl   test2 */
	SMPS("8058_s0", 0x004, 0x084),
	SMPS("8058_s1", 0x005, 0x085),
	SMPS("8058_s2", 0x110, 0x119),
	SMPS("8058_s3", 0x111, 0x11A),
	SMPS("8058_s4", 0x112, 0x11B),

	/*   name        ctrl  */
	LVS("8058_lvs0", 0x12D),
	LVS("8058_lvs1", 0x12F),
};

static int pm8058_vreg_write(struct pm8058_chip *chip,
		u16 addr, u8 val, u8 mask, u8 *reg_save)
{
	int rc;
	u8 reg;

	reg = (*reg_save & ~mask) | (val & mask);
	rc = pm8058_write(chip, addr, &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write failed\n", __func__);
	else
		*reg_save = reg;
	return rc;
}

static int pm8058_vreg_enable(struct regulator_dev *dev)
{
	struct pm8058_vreg *vreg = rdev_get_drvdata(dev);
	struct pm8058_chip *chip = dev_get_drvdata(vreg->rdev->dev.parent);
	int rc = 0;

	rc = pm8058_vreg_write(chip, vreg->ctrl_addr,
			REGULATOR_EN_MASK, REGULATOR_EN_MASK, &vreg->ctrl_reg);
	if (rc)
		pr_err("%s: pm8058_vreg_write failed\n", __func__);

	return rc;
}

static int pm8058_vreg_disable(struct regulator_dev *dev)
{
	struct pm8058_vreg *vreg = rdev_get_drvdata(dev);
	struct pm8058_chip *chip = dev_get_drvdata(vreg->rdev->dev.parent);
	int rc = 0;

	rc = pm8058_vreg_write(chip, vreg->ctrl_addr, 0,
			REGULATOR_EN_MASK, &vreg->ctrl_reg);
	if (rc)
		pr_err("%s: pm8058_vreg_write failed\n", __func__);

	return rc;
}

static int pm8058_pldo_set_voltage(struct pm8058_chip *chip,
		struct pm8058_vreg *vreg, int uV)
{
	int min, max, step, fine_step, rc;
	u8 range_extn, vref, mask, val = 0;

	if (uV >= PLDO_LOW_UV_MIN &&
			uV <= PLDO_LOW_UV_MAX + PLDO_LOW_UV_STEP) {
		min = PLDO_LOW_UV_MIN;
		max = PLDO_LOW_UV_MAX;
		step = PLDO_LOW_UV_STEP;
		fine_step = PLDO_LOW_FINE_STEP_UV;
		range_extn = 0;
		vref = LDO_TEST_VREF_MASK;
	} else if (uV >= PLDO_NORM_UV_MIN &&
			uV <= PLDO_NORM_UV_MAX + PLDO_NORM_UV_STEP) {
		min = PLDO_NORM_UV_MIN;
		max = PLDO_NORM_UV_MAX;
		step = PLDO_NORM_UV_STEP;
		fine_step = PLDO_NORM_FINE_STEP_UV;
		range_extn = 0;
		vref = 0;
	} else {
		min = PLDO_HIGH_UV_MIN;
		max = PLDO_HIGH_UV_MAX;
		step = PLDO_HIGH_UV_STEP;
		fine_step = PLDO_HIGH_FINE_STEP_UV;
		range_extn = LDO_TEST_RANGE_EXTN_MASK;
		vref = 0;
	}

	if (uV > max) {
		uV -= fine_step;
		val = LDO_TEST_FINE_STEP_MASK;
	}

	/* update reference voltage and fine step selection if necessary */
	if ((vreg->test_reg & LDO_TEST_FINE_STEP_MASK) != val ||
			(vreg->test_reg & LDO_TEST_VREF_MASK) != vref) {
		val |= REGULATOR_BANK_SEL(2) | REGULATOR_BANK_WRITE |
			LDO_TEST_VREF_UPDATE_MASK | vref;
		mask = LDO_TEST_VREF_MASK | LDO_TEST_FINE_STEP_MASK | val;

		rc = pm8058_vreg_write(chip, vreg->test_addr, val,
				mask, &vreg->test_reg);
		if (rc) {
			pr_err("%s: pm8058_write failed\n", __func__);
			return rc;
		}
	}

	/* update range extension if necessary */
	if ((vreg->test_reg2 & LDO_TEST_RANGE_EXTN_MASK) != range_extn) {
		val = REGULATOR_BANK_SEL(4) | REGULATOR_BANK_WRITE |
			range_extn;
		mask = LDO_TEST_RANGE_EXTN_MASK | val;

		rc = pm8058_vreg_write(chip, vreg->test_addr, val,
				mask, &vreg->test_reg2);
		if (rc) {
			pr_err("%s: pm8058_write failed\n", __func__);
			return rc;
		}
	}

	/* voltage programming */
	val = (uV - min) / step;
	rc = pm8058_vreg_write(chip, vreg->ctrl_addr, val,
			LDO_CTRL_VPROG_MASK, &vreg->ctrl_reg);
	if (rc)
		pr_err("%s: pm8058_write failed\n", __func__);

	return rc;
}
static int pm8058_nldo_set_voltage(struct pm8058_chip *chip,
		struct pm8058_vreg *vreg, int uV)
{
	int rc;
	u8 mask, val = 0;

	if (uV > NLDO_UV_MAX) {
		uV -= NLDO_FINE_STEP_UV;
		val = LDO_TEST_FINE_STEP_MASK;
	}

	/* update reference voltage and fine step selection if necessary */
	if ((vreg->test_reg & LDO_TEST_FINE_STEP_MASK) != val) {
		val |= REGULATOR_BANK_SEL(2) | REGULATOR_BANK_WRITE;
		mask = LDO_TEST_FINE_STEP_MASK | val;

		rc = pm8058_vreg_write(chip, vreg->test_addr, val,
				mask, &vreg->test_reg);
		if (rc) {
			pr_err("%s: pm8058_write failed\n", __func__);
			return rc;
		}
	}

	/* voltage programming */
	val = (uV - NLDO_UV_MIN) / NLDO_UV_STEP;
	rc = pm8058_vreg_write(chip, vreg->ctrl_addr, val,
			LDO_CTRL_VPROG_MASK, &vreg->ctrl_reg);
	if (rc)
		pr_err("%s: pm8058_write failed\n", __func__);

	return rc;
}

static int pm8058_ldo_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV)
{
	struct pm8058_vreg *vreg = rdev_get_drvdata(dev);
	struct pm8058_chip *chip = dev_get_drvdata(vreg->rdev->dev.parent);
	int uV = (min_uV + max_uV) / 2;

	if (vreg->is_nmos)
		return pm8058_nldo_set_voltage(chip, vreg, uV);
	else
		return pm8058_pldo_set_voltage(chip, vreg, uV);
}

static int pm8058_smps_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV)
{
	struct pm8058_vreg *vreg = rdev_get_drvdata(dev);
	struct pm8058_chip *chip = dev_get_drvdata(vreg->rdev->dev.parent);
	int rc, uV = (min_uV + max_uV) / 2;
	u8 val, mask, vlow;

	if (uV >= SMPS_MODE3_UV_MIN && uV <= SMPS_MODE3_UV_MAX) {
		vlow = SMPS_VLOW_SEL_MASK;
		val = ((uV  - SMPS_MODE3_UV_MIN) / SMPS_MODE3_STEP) |
			SMPS_VREF_SEL_MASK;
	} else if (uV >= SMPS_MODE2_UV_MIN && uV <= SMPS_MODE2_UV_MAX) {
		vlow = 0;
		val = ((uV - SMPS_MODE2_UV_MIN) / SMPS_MODE2_STEP) |
			SMPS_VREF_SEL_MASK;
	} else if (uV >= SMPS_MODE1_UV_MIN && uV <= SMPS_MODE1_UV_MAX) {
		vlow = 0;
		val = (uV - SMPS_MODE1_UV_MIN) / SMPS_MODE1_STEP;
	} else {
		return -EINVAL;
	}

	/* set vlow bit for ultra low voltage mode */
	if ((vreg->test_reg & SMPS_VLOW_SEL_MASK) != vlow) {
		vlow |= REGULATOR_BANK_WRITE | REGULATOR_BANK_SEL(1);
		mask = vlow | SMPS_VLOW_SEL_MASK;
		rc = pm8058_vreg_write(chip, vreg->test2_addr,
				vlow, mask, &vreg->test_reg);
	}

	/* voltage setting */
	mask = SMPS_VREF_SEL_MASK | SMPS_VPROG_MASK;
	rc = pm8058_vreg_write(chip, vreg->ctrl_addr, val, mask,
			&vreg->ctrl_reg);
	if (rc)
		pr_err("%s: pm8058_vreg_write failed\n", __func__);

	return rc;
}

static struct regulator_ops pm8058_ldo_ops = {
	.enable = pm8058_vreg_enable,
	.disable = pm8058_vreg_disable,
	.set_voltage = pm8058_ldo_set_voltage,
};

static struct regulator_ops pm8058_smps_ops = {
	.enable = pm8058_vreg_enable,
	.disable = pm8058_vreg_disable,
	.set_voltage = pm8058_smps_set_voltage,
};

static struct regulator_ops pm8058_lvs_ops = {
	.enable = pm8058_vreg_enable,
	.disable = pm8058_vreg_disable,
};

static int pm8058_init_regulator(struct pm8058_chip *chip,
		struct pm8058_vreg *vreg)
{
	int rc = 0;
	u8 bank;

	/* save the current control register state */
	rc = pm8058_read(chip, vreg->ctrl_addr, &vreg->ctrl_reg, 1);
	if (rc)
		return rc;

	/* save the current test/test2 register state */
	if (vreg->type == REGULATOR_TYPE_LDO) {
		bank = REGULATOR_BANK_SEL(2);
		rc = pm8058_write(chip, vreg->test_addr, &bank, 1);
		if (rc)
			goto bail;

		rc = pm8058_read(chip, vreg->test_addr, &vreg->test_reg, 1);
		if (rc)
			goto bail;

		bank = REGULATOR_BANK_SEL(4);
		rc = pm8058_write(chip, vreg->test_addr, &bank, 1);
		if (rc)
			goto bail;

		rc = pm8058_read(chip, vreg->test_addr, &vreg->test_reg2, 1);
		if (rc)
			goto bail;
	} else if (vreg->type == REGULATOR_TYPE_SMPS) {
		bank = REGULATOR_BANK_SEL(1);
		rc = pm8058_write(chip, vreg->test2_addr, &bank, 1);
		if (rc)
			goto bail;
		rc = pm8058_read(chip, vreg->test2_addr, &vreg->test_reg, 1);
		if (rc)
			goto bail;
	}

bail:
	if (rc)
		pr_err("%s: pm8058_read/write failed\n", __func__);

	return rc;
}

static int pm8058_register_regulator(struct pm8058_chip *chip,
		struct device *dev, int id)
{
	struct pm8058_vreg_pdata *pdata = dev->platform_data;
	struct pm8058_vreg *vreg = &pm8058_vreg[id];
	struct regulator_desc *rdesc = &vreg->rdesc;
	struct regulator_init_data *rdata = &vreg->rdata;
	int rc = 0;

	rdesc->name = vreg->name;
	rdesc->id = id;
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->owner = THIS_MODULE;
	switch (vreg->type) {
	case REGULATOR_TYPE_LDO:
		rdesc->ops = &pm8058_ldo_ops;
		break;
	case REGULATOR_TYPE_SMPS:
		rdesc->ops = &pm8058_smps_ops;
		break;
	case REGULATOR_TYPE_LVS:
		rdesc->ops = &pm8058_lvs_ops;
		break;
	}

	if (pdata) {
		rdata->constraints.min_uV = pdata->min_uV;
		rdata->constraints.max_uV = pdata->max_uV;
	}
	rdata->consumer_supplies = &vreg->rsupply;

	rc = pm8058_init_regulator(chip, vreg);
	if (rc)
		return rc;

	vreg->rdev = regulator_register(rdesc, dev, rdata, vreg);
	if (IS_ERR(vreg->rdev))
		rc = PTR_ERR(vreg->rdev);

	return rc;
}

static int __devinit pm8058_vreg_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (pdev == NULL)
		return -EINVAL;

	if (pdev->id >= 0 && pdev->id < PM8058_VREG_MAX)
		rc = pm8058_register_regulator(platform_get_drvdata(pdev),
				&pdev->dev, pdev->id);
	else
		rc = -ENODEV;

	pr_info("%s: id=%d, rc=%d\n", __func__, pdev->id, rc);

	return rc;
}

static int __devexit pm8058_vreg_remove(struct platform_device *pdev)
{
	regulator_unregister(pm8058_vreg[pdev->id].rdev);
	return 0;
}

static struct platform_driver pm8058_vreg_driver = {
	.probe = pm8058_vreg_probe,
	.remove = __devexit_p(pm8058_vreg_remove),
	.driver = {
		.name = "pm8058-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init pm8058_vreg_init(void)
{
	return platform_driver_register(&pm8058_vreg_driver);
}

static void __exit pm8058_vreg_exit(void)
{
	platform_driver_unregister(&pm8058_vreg_driver);
}

subsys_initcall(pm8058_vreg_init);
module_exit(pm8058_vreg_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("PMIC8058 regulator driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8058-regulator");
