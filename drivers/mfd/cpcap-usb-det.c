/*
 * Copyright (C) 2007 - 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>


#define SENSE_USB           (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_USB_FLASH     (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY       (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_CHARGER_FLOAT (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S)

#define SENSE_CHARGER       (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S)

enum cpcap_det_state {
	CONFIG,
	SAMPLE_1,
	SAMPLE_2,
	IDENTIFY,
	FACTORY,
};

enum cpcap_accy {
	CPCAP_ACCY_USB,
	CPCAP_ACCY_FACTORY,
	CPCAP_ACCY_CHARGER,
	CPCAP_ACCY_NONE,
};

struct cpcap_usb_det_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	struct mutex lock;
	unsigned short sense;
	unsigned short prev_sense;
	enum cpcap_det_state state;
	enum cpcap_accy usb_accy;
};

static struct platform_device cpcap_usb_device = {
	.name              = "cpcap_usb_dev",
	.id                = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_factory_device = {
	.name              = "cpcap_factory",
	.id                = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_charger_device = {
	.name              = "cpcap_charger",
	.id                = -1,
	.dev.platform_data = NULL,
};

static struct platform_device *accy_devices[] = {
	&cpcap_usb_device,
	&cpcap_factory_device,
	&cpcap_charger_device,
};


static int get_sense(struct cpcap_usb_det_data *data)
{
	int retval = -EFAULT;
	unsigned short value;
	struct cpcap_device *cpcap;

	if (!data)
		return -EFAULT;
	cpcap = data->cpcap;

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_GROUND_I),
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_GROUND_I));

	data->sense = value & (CPCAP_BIT_ID_FLOAT_S |
			       CPCAP_BIT_ID_GROUND_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT2,
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_SE1_I),
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_SE1_I));
	if (retval)
		return retval;

	data->sense |= value & (CPCAP_BIT_CHRGCURR1_S |
				CPCAP_BIT_VBUSVLD_S |
				CPCAP_BIT_SESSVLD_S |
				CPCAP_BIT_SE1_S);
	return 0;
}

static int configure_hardware(struct cpcap_usb_det_data *data,
			      enum cpcap_accy accy)
{
	int retval;

	/* Take control of pull up from ULPI. */
	retval  = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     CPCAP_BIT_PU_SPI,
				     CPCAP_BIT_PU_SPI);
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));

	switch (accy) {
	case CPCAP_ACCY_USB:
	case CPCAP_ACCY_FACTORY:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		/* TODO: Call into the regulator framework. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC,
					     CPCAP_BIT_VUSB,
					     CPCAP_BIT_VUSB);
		/* Give USB driver control of pull up via ULPI. */
		retval  = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     0,
					     CPCAP_BIT_PU_SPI);
		break;

	case CPCAP_ACCY_CHARGER:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 0,
					     CPCAP_BIT_USBXCVREN);
		/* TODO: Call into the regulator framework. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC,
					     CPCAP_BIT_VUSB,
					     CPCAP_BIT_VUSB);
		break;

	case CPCAP_ACCY_NONE:
	default:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 0,
					     CPCAP_BIT_USBXCVREN);
		/* TODO: Call into the regulator framework. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC,
					     0,
					     CPCAP_BIT_VUSB);
		break;
	}

	if (retval != 0)
		retval = -EFAULT;

	return retval;
}

static void notify_accy(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	if (data->usb_accy != CPCAP_ACCY_NONE)
		platform_device_unregister(accy_devices[data->usb_accy]);

	configure_hardware(data, accy);
	data->usb_accy = accy;

	if (accy != CPCAP_ACCY_NONE) {
		accy_devices[accy]->dev.platform_data = data->cpcap;
		platform_device_register(accy_devices[accy]);
	}
}

static void detection_work(struct work_struct *work)
{
	struct cpcap_usb_det_data *data =
		container_of(work, struct cpcap_usb_det_data, work.work);

	switch (data->state) {
	case CONFIG:
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);

		configure_hardware(data, CPCAP_ACCY_NONE);

		data->state = SAMPLE_1;
		schedule_delayed_work(&data->work, msecs_to_jiffies(11));
		break;

	case SAMPLE_1:
		get_sense(data);
		data->state = SAMPLE_2;
		schedule_delayed_work(&data->work, msecs_to_jiffies(100));
		break;

	case SAMPLE_2:
		data->prev_sense = data->sense;
		get_sense(data);

		if (data->prev_sense != data->sense) {
			/* Stay in this state */
			data->state = SAMPLE_2;
			schedule_delayed_work(&data->work,
					      msecs_to_jiffies(100));
		} else if (!(data->sense & CPCAP_BIT_SE1_S) &&
			   (data->sense & CPCAP_BIT_ID_FLOAT_S) &&
			   !(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   !(data->sense & CPCAP_BIT_SESSVLD_S)) {
			data->state = IDENTIFY;
			schedule_delayed_work(&data->work,
					      msecs_to_jiffies(100));
		} else {
			data->state = IDENTIFY;
			schedule_delayed_work(&data->work, 0);
		}
		break;

	case IDENTIFY:
		get_sense(data);
		data->state = CONFIG;

		if ((data->sense == SENSE_USB) ||
		    (data->sense == SENSE_USB_FLASH)) {
			notify_accy(data, CPCAP_ACCY_USB);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		} else if (data->sense == SENSE_FACTORY) {
			notify_accy(data, CPCAP_ACCY_FACTORY);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);

			/* Special handling of factory cable undetect. */
			data->state = FACTORY;
		} else if ((data->sense == SENSE_CHARGER_FLOAT) ||
			   (data->sense == SENSE_CHARGER)) {
			notify_accy(data, CPCAP_ACCY_CHARGER);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		} else {
			notify_accy(data, CPCAP_ACCY_NONE);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		}
		break;

	case FACTORY:
		get_sense(data);

		/* The removal of a factory cable can only be detected if a
		 * charger is attached.
		 */
		if (data->sense & CPCAP_BIT_SE1_S) {
			data->state = CONFIG;
			schedule_delayed_work(&data->work, 0);
		} else {
			data->state = FACTORY;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
		}
		break;

	default:
		/* This shouldn't happen.  Need to reset state machine. */
		data->state = CONFIG;
		schedule_delayed_work(&data->work, 0);
		break;
	}
}

static void int_handler(enum cpcap_irqs int_event, void *data)
{
	struct cpcap_usb_det_data *usb_det_data = data;
	schedule_delayed_work(&(usb_det_data->work), 0);
}

static int __init cpcap_usb_det_probe(struct platform_device *pdev)
{
	int retval;
	struct cpcap_usb_det_data *data;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->state = CONFIG;
	platform_set_drvdata(pdev, data);
	INIT_DELAYED_WORK(&data->work, detection_work);
	data->usb_accy = CPCAP_ACCY_NONE;

	retval = configure_hardware(data, CPCAP_ACCY_NONE);

	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_DET,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_CURR1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SE1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDGND,
				     int_handler, data);

	/* Now that HW initialization is done, give USB control via ULPI. */
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     0, CPCAP_BIT_ULPI_SPI_SEL);

	if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "CPCAP USB detection device probed\n");

	/* Perform initial detection */
	detection_work(&(data->work.work));

	return 0;
}

static int __exit cpcap_usb_det_remove(struct platform_device *pdev)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);

	configure_hardware(data, CPCAP_ACCY_NONE);
	cancel_delayed_work_sync(&data->work);

	if (data->usb_accy != CPCAP_ACCY_NONE)
		platform_device_unregister(accy_devices[data->usb_accy]);

	kfree(data);
	return 0;
}


static struct platform_driver cpcap_usb_det_driver = {
	.probe		= cpcap_usb_det_probe,
	.remove		= __exit_p(cpcap_usb_det_remove),
	.driver		= {
		.name	= "cpcap_usb_det",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_det_init(void)
{
	return platform_driver_register(&cpcap_usb_det_driver);
}
module_init(cpcap_usb_det_init);

static void __exit cpcap_usb_det_exit(void)
{
	platform_driver_unregister(&cpcap_usb_det_driver);
}
module_exit(cpcap_usb_det_exit);

MODULE_ALIAS("platform:cpcap_usb_det");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_LICENSE("GPL");
