/*
 * Marimba TSADC driver.
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/marimba.h>
#include <linux/mfd/marimba-tsadc.h>

/* marimba configuration block: TS_CTL0 */
#define TS_CTL0			0xFF
#define TS_CTL0_RESET		BIT(0)
#define TS_CTL0_CLK_EN		BIT(1)
#define TS_CTL0_XO_EN		BIT(2)
#define TS_CTL0_EOC_EN		BIT(3)
#define TS_CTL0_PENIRQ_EN	BIT(4)

/* TSADC registers */
#define SSBI_PRESET		0x00
#define TSHK_DIG_CONFIG		0x4F
#define TSHK_INTF_CONFIG	0x50
#define TSHK_SETUP		0x51
	#define TSHK_SETUP_EN_ADC  BIT(0)
	#define TSHK_SETUP_EN_PIRQ BIT(7)
#define TSHK_PARAM		0x52
#define TSHK_DATA_RD		0x53
#define TSHK_STATUS		0x54
#define TSHK_SETUP2		0x55
#define TSHK_RSV1		0x56
	#define TSHK_RSV1_PRECHARGE_EN	BIT(0)
#define TSHK_COMMAND		0x57
#define TSHK_PARAM2		0x58
	#define TSHK_INPUT_CLK_MASK	0x3F
	#define TSHK_SAMPLE_PRD_MASK	0xC7
	#define TSHK_INPUT_CLK_SHIFT	0x6
	#define TSHK_SAMPLE_PRD_SHIFT	0x3
#define TSHK_PARAM3		0x59
	#define TSHK_PARAM3_MODE_MASK	0xFC
	#define TSHK_PARAM3_PRE_CHG_SHIFT (5)
	#define TSHK_PARAM3_STABIZ_SHIFT (2)
	#define TSHK_STABLE_TIME_MASK	0xE3
	#define TSHK_PRECHG_TIME_MASK	0x1F
#define TSHK_PARAM4		0x5A
#define TSHK_RSV2		0x5B
#define TSHK_RSV3		0x5C
#define TSHK_RSV4		0x5D
#define TSHK_RSV5		0x5E

struct marimba_tsadc_client {
	unsigned int is_ts;
	struct platform_device *pdev;
};

struct marimba_tsadc {
	struct marimba *marimba;
	struct device *dev;
	struct marimba_tsadc_platform_data *pdata;
};

static struct marimba_tsadc *tsadc_dev;

static int marimba_write_u8(struct marimba_tsadc *tsadc, u8 reg, u8 data)
{
	int rc;

	tsadc->marimba->mod_id = MARIMBA_SLAVE_ID_MARIMBA;
	rc = marimba_write(tsadc->marimba, reg, &data, 1);

	if (!rc)
		dev_warn(tsadc->dev, "Error writing marimba reg %X - ret %X\n",
				reg, data);
	return 0;
}

static int marimba_tsadc_write(struct marimba_tsadc *tsadc, u8 reg, u8 data)
{
	int rc;

	tsadc->marimba->mod_id = MARIMBA_ID_TSADC;

	rc = marimba_ssbi_write(tsadc->marimba, reg, &data, 1);
	if (!rc)
		dev_warn(tsadc->dev, "Error writing marimba reg %X - ret %X\n",
				reg, data);
	return rc;
}

static int marimba_tsadc_shutdown(struct marimba_tsadc *tsadc)
{
	u8 val;

	/* force reset */
	val = TS_CTL0_XO_EN | TS_CTL0_EOC_EN | TS_CTL0_PENIRQ_EN |
				TS_CTL0_CLK_EN;
	marimba_write_u8(tsadc, TS_CTL0, val);

	/* disable xo, clock */
	val = TS_CTL0_PENIRQ_EN | TS_CTL0_EOC_EN;
	marimba_write_u8(tsadc, TS_CTL0, val);

	return 0;
}

static int marimba_tsadc_startup(struct marimba_tsadc *tsadc)
{
	u8 val;

	/* disable XO, clock and output enables */
	marimba_write_u8(tsadc, TS_CTL0, 0x00);

	/* Enable output enables */
	val = TS_CTL0_XO_EN | TS_CTL0_EOC_EN | TS_CTL0_PENIRQ_EN;
	marimba_write_u8(tsadc, TS_CTL0, val);

	/* Enable clock */
	val = val | TS_CTL0_CLK_EN;
	marimba_write_u8(tsadc, TS_CTL0, val);

	/* remove reset */
	val = val | TS_CTL0_RESET;
	marimba_write_u8(tsadc, TS_CTL0, val);

	return 0;
}


static int marimba_tsadc_configure(struct marimba_tsadc *tsadc)
{
	u8 rsv1 = 0,  setup = 0, i, count = 0;
	u8 param2 = 0,  param3 = 0;
	unsigned long val;

	marimba_tsadc_write(tsadc, SSBI_PRESET, 0x00);

	if (!tsadc->pdata)
		return -EINVAL;

	/* Configure RSV1 register*/
	if (tsadc->pdata->tsadc_prechg_en == true)
		rsv1 |= TSHK_RSV1_PRECHARGE_EN;
	else
		rsv1 &= ~TSHK_RSV1_PRECHARGE_EN;

	/*  Set RSV1 register*/
	marimba_tsadc_write(tsadc, TSHK_RSV1, rsv1);

	/* Configure PARAM2 register */
	/* Input clk */
	val = tsadc->pdata->params2.input_clk_khz;
	param2 &= TSHK_INPUT_CLK_MASK;
	val /= 600;
	if (val >= 1 && val <= 8 && !(val & (val - 1))) {
		/* Input clk can be .6, 1.2, 2.4, 4.8Mhz */
		if (val % 4 != 0)
			param2 = (4 - (val % 4)) << TSHK_INPUT_CLK_SHIFT;
		else
			param2 = ((val / 4) - 1) << TSHK_INPUT_CLK_SHIFT;
	} else /* Configure the default clk 2.4Mhz */
		param2 = 0x00 << TSHK_INPUT_CLK_SHIFT;

	/* Sample period */
	param2 &= TSHK_SAMPLE_PRD_MASK;
	param2 |=  tsadc->pdata->params2.sample_prd << TSHK_SAMPLE_PRD_SHIFT;

	/* Write PARAM2 register */
	marimba_tsadc_write(tsadc, TSHK_PARAM2, param2);

	/* REVISIT: If Precharge time, stabilization time  > 409.6us */
	/* Configure PARAM3 register */
	val = tsadc->pdata->params3.prechg_time_nsecs;
	param3 &= TSHK_PRECHG_TIME_MASK;
	val /= 6400;
	if (val >= 1 && val <= 64  && !(val & (val - 1))) {
		count = 0;
		while ((val = val >> 1) != 0)
			count++;
		param3 |= count << TSHK_PARAM3_PRE_CHG_SHIFT;
	} else	/* Set default value if the input is wrong */
		param3 |= 0x00 << TSHK_PARAM3_PRE_CHG_SHIFT;

	val = tsadc->pdata->params3.stable_time_nsecs;
	param3 &= TSHK_STABLE_TIME_MASK;
	val /= 6400;
	if (val >= 1 && val <= 64 && !(val & (val - 1))) {
		count = 0;
		while ((val = val >> 1) != 0)
			count++;
		param3 |= count << TSHK_PARAM3_STABIZ_SHIFT;
	} else /* Set default value if the input is wrong */
		param3 |=  0x00 << TSHK_PARAM3_STABIZ_SHIFT;

	/* Get TSADC mode */
	val = tsadc->pdata->params3.tsadc_test_mode;
	param3 &= TSHK_PARAM3_MODE_MASK;
	if (val == 0)
		param3 |= 0x00;
	else
		for (i = 0; i < 3 ; i++) {
			if (((val + i) % 39322) == 0) {
				param3 |= (i + 1);
				break;
			}
		}
	if (i == 3) /* Set to normal mode if input is wrong */
		param3 |= 0x00;

	marimba_tsadc_write(tsadc, TSHK_PARAM3, param3);

	/* Configure TSHK SETUP Register */
	if (tsadc->pdata->setup.pen_irq_en == true)
		setup |= TSHK_SETUP_EN_PIRQ;
	else
		setup &= ~TSHK_SETUP_EN_PIRQ;

	if (tsadc->pdata->setup.tsadc_en == true)
		setup |= TSHK_SETUP_EN_ADC;
	else
		setup &= ~TSHK_SETUP_EN_ADC;

	/* Enable signals to ADC, pen irq assertion */
	marimba_tsadc_write(tsadc, TSHK_SETUP, setup);

	return 0;
}

int marimba_tsadc_start(struct marimba_tsadc_client *client)
{
	if (!client) {
		pr_err("%s: Not a valid client\n", __func__);
		return -ENODEV;
	}

	if (!tsadc_dev) {
		dev_err(&client->pdev->dev,
			"%s: No tsadc device available\n", __func__);
		return -ENODEV;
	}

	/* REVISIT - add locks */
	if (client->is_ts) {
		marimba_tsadc_startup(tsadc_dev);
		marimba_tsadc_configure(tsadc_dev);
	}

	return 0;
}
EXPORT_SYMBOL(marimba_tsadc_start);

struct marimba_tsadc_client *
marimba_tsadc_register(struct platform_device *pdev, unsigned int is_ts)
{
	struct marimba_tsadc_client *client;

	if (!pdev) {
		pr_err("%s: valid platform device pointer please\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	if (!is_ts) {
		dev_err(&pdev->dev, "%s: only TS right now\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	if (!tsadc_dev) {
		dev_err(&pdev->dev,
			"%s: No tsadc device available\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->pdev = pdev;
	client->is_ts = is_ts;

	return client;
}
EXPORT_SYMBOL(marimba_tsadc_register);

void marimba_tsadc_unregister(struct marimba_tsadc_client *client)
{
	if (client->is_ts)
		marimba_tsadc_shutdown(tsadc_dev);
	kfree(client);
}
EXPORT_SYMBOL(marimba_tsadc_unregister);

static int __devinit marimba_tsadc_probe(struct platform_device *pdev)
{
	struct marimba *marimba = platform_get_drvdata(pdev);
	struct marimba_tsadc *tsadc;
	int rc = 0;

	printk("%s\n", __func__);

	tsadc = kzalloc(sizeof *tsadc, GFP_KERNEL);
	if (!tsadc)
		return -ENOMEM;

	tsadc->marimba	= marimba;
	tsadc->dev	= &pdev->dev;
	tsadc->pdata = pdev->dev.platform_data;

	if (tsadc->pdata->marimba_tsadc_power) {
		rc = tsadc->pdata->marimba_tsadc_power(1);
		if (rc) {
			pr_err("%s: Unable to power up TSADC \n", __func__);
			kfree(tsadc);
			return rc;
		}
	}

	platform_set_drvdata(pdev, tsadc);

	tsadc_dev = tsadc;

	return rc;
}

static int __devexit marimba_tsadc_remove(struct platform_device *pdev)
{
	struct marimba_tsadc *tsadc = platform_get_drvdata(pdev);

	if (tsadc->pdata->marimba_tsadc_power)
		tsadc->pdata->marimba_tsadc_power(0);

	platform_set_drvdata(pdev, NULL);
	kfree(tsadc);
	return 0;
}

#ifdef CONFIG_PM
static int
marimba_tsadc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	int rc = 0;
	struct marimba_tsadc *tsadc = platform_get_drvdata(pdev);

	if (tsadc->pdata->marimba_tsadc_power)
		rc = tsadc->pdata->marimba_tsadc_power(0);

	return rc;
}

static int marimba_tsadc_resume(struct platform_device *pdev)
{
	int rc = 0;
	struct marimba_tsadc *tsadc = platform_get_drvdata(pdev);

	if (tsadc->pdata->marimba_tsadc_power) {
		rc = tsadc->pdata->marimba_tsadc_power(1);
		if (rc) {
			pr_err("%s: Unable to power on TSADC \n", __func__);
			return rc;
		}
	}

	marimba_tsadc_startup(tsadc_dev);
	marimba_tsadc_configure(tsadc_dev);

	return rc;
}
#else
static int
marimba_tsadc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int marimba_tsadc_resume(struct platform_device *pd)
{
	return 0;
}
#endif

static struct platform_driver tsadc_driver = {
	.probe	= marimba_tsadc_probe,
	.remove	= __devexit_p(marimba_tsadc_remove),
	.driver	= {
		.name = "marimba_tsadc",
		.owner = THIS_MODULE,
	},
	.resume = marimba_tsadc_resume,
	.suspend = marimba_tsadc_suspend,
};

static int __init marimba_tsadc_init(void)
{
	return platform_driver_register(&tsadc_driver);
}
device_initcall(marimba_tsadc_init);

static void __exit marimba_tsadc_exit(void)
{
	return platform_driver_unregister(&tsadc_driver);
}
device_exit(marimba_tsadc_exit);

MODULE_DESCRIPTION("Marimba TSADC driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:marimba_tsadc");
