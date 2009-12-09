/*
 * Copyright (C) 2007-2009 Motorola, Inc.
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

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/delay.h>

static int ioctl(struct inode *inode,
		 struct file *file, unsigned int cmd, unsigned long arg);
static int __devinit cpcap_probe(struct spi_device *spi);
static int __devexit cpcap_remove(struct spi_device *spi);

const static struct file_operations cpcap_fops = {
	.owner = THIS_MODULE,
	.ioctl = ioctl,
};

static struct miscdevice cpcap_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= CPCAP_DEV_NAME,
	.fops	= &cpcap_fops,
};

static struct spi_driver cpcap_driver = {
	.driver = {
		   .name = "cpcap",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = cpcap_probe,
	.remove = __devexit_p(cpcap_remove),
};

static struct platform_device cpcap_adc_device = {
	.name           = "cpcap_adc",
	.id             = -1,
	.dev.platform_data = NULL,
};


static struct platform_device cpcap_key_device = {
	.name           = "cpcap_key",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_batt_device = {
	.name           = "cpcap_battery",
	.id             = -1,
	.dev.platform_data = NULL,
};

struct platform_device cpcap_disp_button_led = {
	.name		= LD_DISP_BUTTON_DEV,
	.id		= -1,
	.dev		= {
		.platform_data  = NULL,
	},
};

struct platform_device cpcap_rgb_led = {
	.name		= LD_MSG_IND_DEV,
	.id		= -1,
	.dev		= {
		.platform_data  = NULL,
	},
};

struct platform_device cpcap_keypad_led = {
	.name		= LD_KPAD_DEV,
	.id		= -1,
	.dev		= {
		.platform_data  = NULL,
	},
};

struct platform_device cpcap_lm3554 = {
	.name		= "flash-torch",
	.id		= -1,
	.dev		= {
		.platform_data  = NULL,
	},
};

#ifdef CONFIG_CPCAP_USB
static struct platform_device cpcap_usb_device = {
	.name           = "cpcap_usb",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_usb_det_device = {
	.name           = "cpcap_usb_det",
	.id             = -1,
	.dev.platform_data = NULL,
};
#endif

#ifdef CONFIG_SOUND_CPCAP_OMAP
static struct platform_device cpcap_audio_device = {
	.name           = "cpcap_audio",
	.id             = -1,
	.dev.platform_data  = NULL,
};
#endif

static struct platform_device cpcap_uc_device = {
	.name           = "cpcap_uc",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_3mm5_device = {
	.name           = "cpcap_3mm5",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_rtc_device = {
	.name           = "cpcap_rtc",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device *cpcap_devices[] = {
	&cpcap_adc_device,
	&cpcap_key_device,
	&cpcap_batt_device,
	&cpcap_disp_button_led,
	&cpcap_rgb_led,
	&cpcap_keypad_led,
	&cpcap_lm3554,
#ifdef CONFIG_CPCAP_USB
	&cpcap_usb_device,
	&cpcap_usb_det_device,
#endif
#ifdef CONFIG_SOUND_CPCAP_OMAP
	&cpcap_audio_device,
#endif
	&cpcap_3mm5_device,
	&cpcap_rtc_device,
	&cpcap_uc_device,
};

static struct cpcap_device *misc_cpcap;

static int cpcap_reboot(struct notifier_block *this, unsigned long code,
			void *cmd)
{
	int ret = -1;
	int result = NOTIFY_DONE;
	char *mode = cmd;

	/* Disable the USB transceiver */
	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_USBC2, 0,
				 CPCAP_BIT_USBXCVREN);

	if (ret) {
		dev_err(&(misc_cpcap->spi->dev),
			"Disable Transciever failure.\n");
		result = NOTIFY_BAD;
	}

	if (code == SYS_RESTART) {
		/* Set the soft reset bit in the cpcap */
		ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1,
				CPCAP_BIT_SOFT_RESET, CPCAP_BIT_SOFT_RESET);
		if (ret) {
			dev_err(&(misc_cpcap->spi->dev),
				"SW Reset cpcap set failure.\n");
			result = NOTIFY_BAD;
		}

		/* Check if we are starting recovery mode */
		if (mode != NULL && !strncmp("recovery", mode, 9)) {
			/* Set the fota (recovery mode) bit in the cpcap */
			ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1,
				CPCAP_BIT_FOTA_MODE, CPCAP_BIT_FOTA_MODE);
			if (ret) {
				dev_err(&(misc_cpcap->spi->dev),
					"Recovery cpcap set failure.\n");
				result = NOTIFY_BAD;
			}
		} else {
			/* Set the fota (recovery mode) bit in the cpcap */
			ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1, 0,
						CPCAP_BIT_FOTA_MODE);
			if (ret) {
				dev_err(&(misc_cpcap->spi->dev),
					"Recovery cpcap clear failure.\n");
				result = NOTIFY_BAD;
			}
		}
		/* Check if we are going into fast boot mode */
		if (mode != NULL && !strncmp("bootloader", mode, 11)) {
			/* Set the bootmode bit in the cpcap */
			ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1,
				CPCAP_BIT_BOOT_MODE, CPCAP_BIT_BOOT_MODE);
			if (ret) {
				dev_err(&(misc_cpcap->spi->dev),
					"Boot mode cpcap set failure.\n");
				result = NOTIFY_BAD;
			}
		}
		cpcap_regacc_write(misc_cpcap, CPCAP_REG_MI2, 0, 0xFFFF);
	} else {
		/* Clear the soft reset bit in the cpcap */
		ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1, 0,
					CPCAP_BIT_SOFT_RESET);
		if (ret) {
			dev_err(&(misc_cpcap->spi->dev),
				"SW Reset cpcap set failure.\n");
			result = NOTIFY_BAD;
		}
		/* Clear the fota (recovery mode) bit in the cpcap */
		ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1, 0,
					CPCAP_BIT_FOTA_MODE);
		if (ret) {
			dev_err(&(misc_cpcap->spi->dev),
				"Recovery cpcap clear failure.\n");
			result = NOTIFY_BAD;
		}
	}

	/* Always clear the kpanic bit */
	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1,
		0, CPCAP_BIT_AP_KERNEL_PANIC);
	if (ret) {
		dev_err(&(misc_cpcap->spi->dev),
			"Clear kernel panic bit failure.\n");
		result = NOTIFY_BAD;
	}

	/* Always clear the power cut bit on SW Shutdown*/
	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_PC1,
		0, CPCAP_BIT_PC1_PCEN);
	if (ret) {
		dev_err(&(misc_cpcap->spi->dev),
			"Clear Power Cut bit failure.\n");
		result = NOTIFY_BAD;
	}

	/* Clear the charger and charge path settings to avoid a false turn on
	 * event in caused by CPCAP. After clearing these settings, 100ms is
	 * needed to before SYSRSTRTB is pulled low to avoid the false turn on
	 * event.
	 */
	cpcap_regacc_write(misc_cpcap, CPCAP_REG_CRM, 0, 0x3FFF);
	mdelay(100);

	return result;
}
static struct notifier_block cpcap_reboot_notifier = {
	.notifier_call = cpcap_reboot,
};

static int __init cpcap_init(void)
{
	return spi_register_driver(&cpcap_driver);
}

#ifdef CONFIG_CPCAP_USB
static struct regulator_consumer_supply cpcap_vusb_consumers = {
	.supply = "vusb",
	.dev = &cpcap_usb_det_device.dev,
};
#endif

static void cpcap_vendor_read(struct cpcap_device *cpcap)
{
	unsigned short value;

	(void)cpcap_regacc_read(cpcap, CPCAP_REG_VERSC1, &value);

	cpcap->vendor = (enum cpcap_vendor)((value >> 6) & 0x0007);
	cpcap->revision = (enum cpcap_revision)(((value >> 3) & 0x0007) |
						((value << 3) & 0x0038));
}


static int __devinit cpcap_probe(struct spi_device *spi)
{
	int retval = -EINVAL;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *data;
	int i;

	cpcap = kzalloc(sizeof(*cpcap), GFP_KERNEL);
	if (cpcap == NULL)
		return -ENOMEM;

	cpcap->spi = spi;
	data = spi->controller_data;

	misc_cpcap = cpcap;  /* kept for misc device */
	spi_set_drvdata(spi, cpcap);

	retval = cpcap_regacc_init(cpcap);
	if (retval < 0)
		goto free_mem;
	retval = cpcap_irq_init(cpcap);
	if (retval < 0)
		goto free_cpcap_irq;

	/* Set Kpanic bit, which will be cleared at normal reboot */
	cpcap_regacc_write(cpcap, CPCAP_REG_VAL1,
			CPCAP_BIT_AP_KERNEL_PANIC, CPCAP_BIT_AP_KERNEL_PANIC);

	cpcap_vendor_read(cpcap);

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_devices[i]->dev.platform_data = cpcap;

	retval = misc_register(&cpcap_dev);
	if (retval < 0)
		goto free_cpcap_irq;

#ifdef CONFIG_CPCAP_USB
	/* the cpcap usb_detection device is a consumer of the
	 * vusb regulator */
	data->regulator_init[CPCAP_VUSB].num_consumer_supplies = 1;
	data->regulator_init[CPCAP_VUSB].consumer_supplies =
		&cpcap_vusb_consumers;
#endif
	/* loop twice becuase cpcap_regulator_probe may refer to other devices
	 * in this list to handle dependencies between regulators.  Create them
	 * all and then add them */
	for (i = 0; i < CPCAP_NUM_REGULATORS; i++) {
		struct platform_device *pdev;

		pdev = platform_device_alloc("cpcap-regltr", i);
		if (!pdev) {
			dev_err(&(spi->dev), "Cannot create regulator\n");
			continue;
		}

		pdev->dev.parent = &(spi->dev);
		pdev->dev.platform_data = &data->regulator_init[i];
		platform_set_drvdata(pdev, cpcap);
		cpcap->regulator_pdev[i] = pdev;
	}

	for (i = 0; i < CPCAP_NUM_REGULATORS; i++) {
		/* vusb has to be added after sw5 so skip it for now,
		 * it will be added from probe of sw5 */
		if (i == CPCAP_VUSB)
			continue;
		platform_device_add(cpcap->regulator_pdev[i]);
	}

	platform_add_devices(cpcap_devices, ARRAY_SIZE(cpcap_devices));

	register_reboot_notifier(&cpcap_reboot_notifier);

	return 0;

free_cpcap_irq:
	cpcap_irq_shutdown(cpcap);
free_mem:
	kfree(cpcap);
	return retval;
}

static int __devexit cpcap_remove(struct spi_device *spi)
{
	struct cpcap_device *cpcap = spi_get_drvdata(spi);
	int i;

	unregister_reboot_notifier(&cpcap_reboot_notifier);

	for (i = ARRAY_SIZE(cpcap_devices); i > 0; i--)
		platform_device_unregister(cpcap_devices[i-1]);

	for (i = 0; i < CPCAP_NUM_REGULATORS; i++)
		platform_device_unregister(cpcap->regulator_pdev[i]);

	misc_deregister(&cpcap_dev);
	cpcap_irq_shutdown(cpcap);
	kfree(cpcap);
	return 0;
}


static int test_ioctl(unsigned int cmd, unsigned long arg)
{
	int retval = -EINVAL;
	struct cpcap_regacc read_data;
	struct cpcap_regacc write_data;

	switch (cmd) {
	case CPCAP_IOCTL_TEST_READ_REG:
		if (copy_from_user((void *)&read_data, (void *)arg,
				   sizeof(read_data)))
			return -EFAULT;
		retval = cpcap_regacc_read(misc_cpcap, read_data.reg,
					   &read_data.value);
		if (retval < 0)
			return retval;
		if (copy_to_user((void *)arg, (void *)&read_data,
				 sizeof(read_data)))
			return -EFAULT;
		return 0;
	break;

	case CPCAP_IOCTL_TEST_WRITE_REG:
		if (copy_from_user((void *) &write_data,
				   (void *) arg,
				   sizeof(write_data)))
			return -EFAULT;
		retval = cpcap_regacc_write(misc_cpcap, write_data.reg,
					    write_data.value, write_data.mask);
	break;

	default:
		retval = -ENOTTY;
	break;
	}

	return retval;
}

static int adc_ioctl(unsigned int cmd, unsigned long arg)
{
	int retval = -EINVAL;
	struct cpcap_adc_phase phase;

	switch (cmd) {
	case CPCAP_IOCTL_ADC_PHASE:
		if (copy_from_user((void *) &phase, (void *) arg,
				   sizeof(phase)))
			return -EFAULT;

		cpcap_adc_phase(misc_cpcap, &phase);
		retval = 0;
	break;

	default:
		retval = -ENOTTY;
	break;
	}

	return retval;
}

static int ioctl(struct inode *inode,
		 struct file *file, unsigned int cmd, unsigned long arg)
{
	int retval = -ENOTTY;
	unsigned int cmd_num;

	cmd_num = _IOC_NR(cmd);

	if ((cmd_num > CPCAP_IOCTL_NUM_TEST__START) &&
	    (cmd_num < CPCAP_IOCTL_NUM_TEST__END)) {
		retval = test_ioctl(cmd, arg);
	}
	if ((cmd_num > CPCAP_IOCTL_NUM_ADC__START) &&
	    (cmd_num < CPCAP_IOCTL_NUM_ADC__END)) {
		retval = adc_ioctl(cmd, arg);
	}

	return retval;
}

static void cpcap_shutdown(void)
{
	spi_unregister_driver(&cpcap_driver);
}

subsys_initcall(cpcap_init);
module_exit(cpcap_shutdown);

MODULE_ALIAS("platform:cpcap");
MODULE_DESCRIPTION("CPCAP driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
