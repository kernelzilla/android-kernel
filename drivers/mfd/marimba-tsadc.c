/*
 * Marimba TSADC driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#define TSHK_PARAM		0x52
#define TSHK_DATA_RD		0x53
#define TSHK_STATUS		0x54
#define TSHK_SETUP2		0x55
#define TSHK_RSV1		0x56
	#define TSHK_RSV1_PRECHARGE_EN	BIT(0)
#define TSHK_COMMAND		0x57
#define TSHK_PARAM2		0x58
#define TSHK_PARAM3		0x59
	#define TSHK_PARAM3_NORMAL_MODE	BIT(1)
	#define TSHK_PARAM3_PRE_CHG_SHIFT (5)
	#define TSHK_PARAM3_STABIZ_SHIFT (2)
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
	u8 val;

	marimba_tsadc_write(tsadc, SSBI_PRESET, 0x00);
	/* Enable pre-charge */
	marimba_tsadc_write(tsadc, TSHK_RSV1, TSHK_RSV1_PRECHARGE_EN);

	/* Input clock 2.4 MHz and sample period being 1.15ms */
	marimba_tsadc_write(tsadc, TSHK_PARAM2, 0x00);

	/* TSADC normal-mode */
	val = TSHK_PARAM3_NORMAL_MODE;
	/* 102.4us pre-charge time */
	val |=	0x4 << TSHK_PARAM3_PRE_CHG_SHIFT;
	/* 6.4us stabilization time */
	val |=	0x0 << TSHK_PARAM3_STABIZ_SHIFT;

	marimba_tsadc_write(tsadc, TSHK_PARAM3, val);

	return 0;
}

int marimba_tsadc_start(struct marimba_tsadc_client *client)
{
	if (!tsadc_dev)
		dev_err(&client->pdev->dev,
			"%s no tsadc device available\n", __func__);

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

	if (!pdev)
		pr_err("%s: valid platform device pointer please\n", __func__);

	if (!is_ts) {
		dev_err(&pdev->dev, "%s: only TS right now\n", __func__);
		return ERR_PTR(-EINVAL);
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

	tsadc = kzalloc(sizeof *tsadc, GFP_KERNEL);
	if (!tsadc)
		return -ENOMEM;

	tsadc->marimba	= marimba;
	tsadc->dev	= &pdev->dev;
	tsadc->pdata = pdev->dev.platform_data;

	if (tsadc->pdata->marimba_tsadc_power)
		rc = tsadc->pdata->marimba_tsadc_power(1);

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

static struct platform_driver tsadc_driver = {
	.probe	= marimba_tsadc_probe,
	.remove	= __devexit_p(marimba_tsadc_remove),
	.driver	= {
		.name = "marimba_tsadc",
		.owner = THIS_MODULE,
	},
};

static int __init marimba_tsadc_init(void)
{
	return platform_driver_register(&tsadc_driver);
}
module_init(marimba_tsadc_init);

static void __exit marimba_tsadc_exit(void)
{
	return platform_driver_unregister(&tsadc_driver);
}
module_exit(marimba_tsadc_exit);

MODULE_DESCRIPTION("Marimba TSADC driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:marimba_tsadc");
