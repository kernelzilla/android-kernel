/*
 * cpcap_usb - CPCAP USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2009 Google, Inc.
 * Contact: Erik Gilling <konkers@android.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Based on twl4030-usb.c
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>



enum linkstat {
	USB_LINK_UNKNOWN = 0,
	USB_LINK_NONE,
	USB_LINK_VBUS,
	USB_LINK_ID,
};

struct cpcap_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	int			irq;
	u8			linkstat;
	u8			asleep;
	bool			irq_enabled;
};

/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct cpcap_usb, otg);

static int cpcap_set_suspend(struct otg_transceiver *x, int suspend)
{
	return 0;
}

static int cpcap_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct cpcap_usb *cpcap;

	if (!x)
		return -ENODEV;

	cpcap = xceiv_to_twl(x);
	cpcap->otg.gadget = gadget;
	if (!gadget)
		cpcap->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int cpcap_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct cpcap_usb *cpcap;

	if (!x)
		return -ENODEV;

	cpcap = xceiv_to_twl(x);
	cpcap->otg.host = host;
	if (!host)
		cpcap->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int __init cpcap_usb_probe(struct platform_device *pdev)
{
	struct cpcap_usb	*cpcap;

	cpcap = kzalloc(sizeof *cpcap, GFP_KERNEL);
	if (!cpcap)
		return -ENOMEM;

	cpcap->dev			= &pdev->dev;
	cpcap->otg.dev			= cpcap->dev;
	cpcap->otg.label		= "cpcap";
	cpcap->otg.set_host		= cpcap_set_host;
	cpcap->otg.set_peripheral	= cpcap_set_peripheral;
	cpcap->otg.set_suspend		= cpcap_set_suspend;
	cpcap->asleep			= 1;

	/* init spinlock for workqueue */
	spin_lock_init(&cpcap->lock);

	otg_set_transceiver(&cpcap->otg);

	platform_set_drvdata(pdev, cpcap);

	dev_info(&pdev->dev, "Initialized CPCAP USB module\n");
	return 0;
}

static int __exit cpcap_usb_remove(struct platform_device *pdev)
{
	struct cpcap_usb *cpcap = platform_get_drvdata(pdev);
	kfree(cpcap);

	return 0;
}

static struct platform_driver cpcap_usb_driver = {
	.probe		= cpcap_usb_probe,
	.remove		= __exit_p(cpcap_usb_remove),
	.driver		= {
		.name	= "cpcap_usb",
		.owner	= THIS_MODULE,
	},
};


/*
 * stub CPCAP driver to let us disable SPI control over
 * ULPI bits.  This will go away when the real CPCAP
 * driver is imported
 */

#define CPCAP_REG_SPI_ULPI            898

#define CPCAP_VBUSEN_SPI              0x00000080
#define CPCAP_VBUSPU_SPI              0x00000040
#define CPCAP_VBUSPD_SPI              0x00000020
#define CPCAP_DMPD_SPI                0x00000010
#define CPCAP_DPPD_SPI                0x00000008
#define CPCAP_SUSPEND_SPI             0x00000004
#define CPCAP_PU_SPI                  0x00000002
#define CPCAP_ULPI_SPI_SEL            0x00000001

static int cpcap_read(struct spi_device *spi, unsigned addr, u16 *data)
{
	/* force x_buf to be 32bit aligned */
	u32 buf;
	u8 *x_buf = (u8 *) &buf;
	int r;
	struct spi_message	 m;
	struct spi_transfer  x;

	x_buf[3] = (addr >> 6) & 0x000000FF;
	x_buf[2] = (addr << 2) & 0x000000FF;
	x_buf[1] = 0;
	x_buf[0] = 0;

	spi_message_init(&m);

	memset(&x, 0, sizeof(x));
	x.tx_buf = x_buf;
	x.rx_buf = x_buf;
	x.bits_per_word = 32;
	x.len = 4;
	spi_message_add_tail(&x, &m);

	r = spi_sync(spi, &m);
	if (r < 0)
		return r;

	*data = x_buf[0] | (x_buf[1]<<8);

	return 0;
}

static int cpcap_write(struct spi_device *spi, unsigned addr, u16 data)
{
	/* force x_buf to be 32bit aligned */
	u32 buf;
	u8 *x_buf = (u8 *) &buf;
	int r;
	struct spi_message	 m;
	struct spi_transfer  x;

	x_buf[3] = ((addr >> 6) & 0x000000FF) | 0x80;
	x_buf[2] = (addr << 2) & 0x000000FF;
	x_buf[1] = (data >> 8) & 0x000000FF;
	x_buf[0] = data & 0x000000FF;

	spi_message_init(&m);

	memset(&x, 0, sizeof(x));
	x.tx_buf = x_buf;
	x.bits_per_word = 32;
	x.len = 4;
	spi_message_add_tail(&x, &m);

	r = spi_sync(spi, &m);
	if (r < 0)
		return r;

	return 0;
}


static int cpcap_spi_probe(struct spi_device *spi)
{
	u16 data;
	int r;

	/* disable SPI control over ULPI bits */
	r = cpcap_read(spi, CPCAP_REG_SPI_ULPI, &data);
	if (r < 0) {
		dev_err(&spi->dev,
			"Can't disable SPI control of CPCAP transceiver\n");
		return r;
	}

	data &= ~(CPCAP_VBUSEN_SPI | CPCAP_VBUSPU_SPI | CPCAP_VBUSPD_SPI |
		  CPCAP_DMPD_SPI | CPCAP_DPPD_SPI | CPCAP_SUSPEND_SPI |
		  CPCAP_PU_SPI | CPCAP_ULPI_SPI_SEL);

	r = cpcap_write(spi, CPCAP_REG_SPI_ULPI, data);
	if (r < 0) {
		dev_err(&spi->dev,
			"Can't disable SPI control of CPCAP transceiver\n");
		return r;
	}
	return platform_driver_register(&cpcap_usb_driver);
}

static int cpcap_spi_remove(struct spi_device *spi)
{
	platform_driver_unregister(&cpcap_usb_driver);
	return 0;
}


static struct spi_driver cpcap_spi_driver = {
	.driver = {
		.name	= "cpcap",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= cpcap_spi_probe,
	.remove	= __devexit_p(cpcap_spi_remove),
};


static int __init cpcap_usb_init(void)
{
	return spi_register_driver(&cpcap_spi_driver);
}
subsys_initcall(cpcap_usb_init);

static void __exit cpcap_usb_exit(void)
{
	spi_unregister_driver(&cpcap_spi_driver);
}
module_exit(cpcap_usb_exit);




MODULE_ALIAS("platform:cpcap_usb");
MODULE_DESCRIPTION("CPCAP USB transceiver driver");
MODULE_LICENSE("GPL");
