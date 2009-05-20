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
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/uaccess.h>


static int ioctl(struct inode *inode,
		 struct file *file, unsigned int cmd, unsigned long arg);
static int __devinit cpcap_probe(struct spi_device *spi);
static int __devexit cpcap_remove(struct spi_device *spi);
static int test_ioctl(unsigned int cmd, unsigned long arg);

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

static struct platform_device cpcap_key_device = {
	.name           = "cpcap_key",
	.id             = -1,
	.dev.platform_data = NULL,
};

#ifdef CONFIG_CPCAP_USB
static struct platform_device cpcap_usb_device = {
	.name           = "cpcap_usb",
	.id             = -1,
	.dev.platform_data = NULL,
};
#endif

static struct platform_device *cpcap_devices[] __initdata = {
	&cpcap_key_device,
#ifdef CONFIG_CPCAP_USB
	&cpcap_usb_device,
#endif
};

static struct cpcap_device *misc_cpcap;


static __init int cpcap_init(void)
{
	return spi_register_driver(&cpcap_driver);
}

static int __devinit cpcap_probe(struct spi_device *spi)
{
	int retval = -EINVAL;
	struct cpcap_device *cpcap;
	int i;

	cpcap = kzalloc(sizeof(*cpcap), GFP_KERNEL);
	if (cpcap == NULL)
		return -ENOMEM;

	cpcap->spi = spi;
	misc_cpcap = cpcap;  /* kept for misc device */
	spi_set_drvdata(spi, cpcap);

	retval = cpcap_regacc_init(cpcap);
	if (retval < 0)
		return retval;
	retval = cpcap_irq_init(cpcap);
	if (retval < 0)
		return retval;

	cpcap_usb_device.dev.platform_data = cpcap;
	cpcap_key_device.dev.platform_data = cpcap;

	retval = misc_register(&cpcap_dev);
	if (retval < 0)
		return retval;

	platform_add_devices(cpcap_devices, ARRAY_SIZE(cpcap_devices));

	for (i = 0; i < CPCAP_NUM_REGULATORS; i++) {
		struct platform_device *pdev;

		pdev = platform_device_alloc("cpcap-regltr", i);
		if (!pdev) {
			dev_err(&(spi->dev), "Cannot create regulator\n");
			continue;
		}

		pdev->dev.parent = &(spi->dev);
		pdev->dev.driver_data = cpcap;
		cpcap->regulator_pdev[i] = pdev;

		platform_device_add(pdev);
	}

	return retval;
}

static int __devexit cpcap_remove(struct spi_device *spi)
{
	struct cpcap_device *cpcap = spi_get_drvdata(spi);
	int i;

	platform_device_unregister(&cpcap_key_device);
	platform_device_unregister(&cpcap_usb_device);

	for (i = 0; i < CPCAP_NUM_REGULATORS; i++)
		platform_device_unregister(cpcap->regulator_pdev[i]);

	misc_deregister(&cpcap_dev);
	cpcap_irq_shutdown(cpcap);
	kfree(cpcap);
	return 0;
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

	return retval;
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
