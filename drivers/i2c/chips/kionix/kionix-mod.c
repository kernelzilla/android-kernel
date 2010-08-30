/**
 *	Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation version 2 of the License.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	GNU General Public License <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#include "kionix-main.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LaJolla 3-axis accelerometer driver");
MODULE_AUTHOR("Konstantin Makariev <hcv867@motorola.com>");

/*
 *  Global variables definition
 */
accel_data_t accel_info;

int accel_param_major = 0;
int accel_param_minor = 0;
int accel_param_nr_devs = ACCEL_NR_DEVS;
int accel_param_verbose = 1;
int accel_param_debug = 1;
int accel_param_power = 0;
int accel_param_input = 1;

unsigned long accel_events_mask = 0;	/* global events mask: when it's 0 we don't look through list of private data */

module_param(accel_param_major, int, S_IRUGO);
module_param(accel_param_minor, int, S_IRUGO);
module_param(accel_param_nr_devs, int, S_IRUGO);
module_param(accel_param_verbose, int, S_IRUGO);
module_param(accel_param_debug, int, S_IRUGO);
module_param(accel_param_power, int, S_IRUGO);
module_param(accel_param_input, int, S_IRUGO);

static unsigned short normal_i2c[] = {ACCEL_I2C_ADDRESS, I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(accel);

static int accel_attach_adapter(struct i2c_adapter *i2c);
static int accel_detach_client(struct i2c_client *i2c);

static struct i2c_driver accel_i2c_driver = {
	.driver = {
		.name  = MODULE_NAME,
		//.bus   = BUS_I2C,
		.owner = THIS_MODULE,
	},
	.attach_adapter = accel_attach_adapter,
	.detach_client  = accel_detach_client,
//	.suspend = accel_suspend,
//	.resume = accel_resume,
};

static void accel_input_poll(struct input_polled_dev *dev)
{
	struct input_dev *idev = dev->input;
	int axis[3];

//	mutex_lock (&accel_info.mlock);
//	if (accel_info.mode = PWR_ON) {

		accel_i2c_read_axis_data (axis);

		input_report_abs (idev, ABS_X, axis[0]);
		input_report_abs (idev, ABS_Y, axis[1]);
		input_report_abs (idev, ABS_Z, axis[2]);
		input_sync (idev);
//	}
//	mutex_unlock (&accel_info.mlock);
}


/**
 * Detect implementation
 * Inits the character device and completes the I2C driver initialization
 * @return 0 in success, or negative error code
 */
static int accel_detect(struct i2c_adapter *adapter, int address, int kind)
{
	int result;
	int err=-ENOMEM;
	struct input_polled_dev *idev;
	struct cdev *cdev;
	struct i2c_client *new;
	dev_t dev = 0;

	printk("%s: address 0x%x\n", __FUNCTION__, address);

	//gpio_direction_output(32, 1);	// acl_pwrdn_n
	gpio_direction_output(33, 0);	// acl_pwrdn_n

	if (!i2c_check_functionality (adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		goto exit;

	memset (&accel_info, 0, sizeof (accel_data_t));
	accel_info.magic = ACCEL_MAGIC;
	accel_info.irq = -1;

	spin_lock_init (&accel_info.slock);
	mutex_init (&accel_info.mlock);

	new = &accel_info.i2c;
	i2c_set_clientdata (new, &accel_info);
	new->addr = address;
	new->adapter = adapter;
	new->driver = &accel_i2c_driver;
	new->flags = 0;

	printk ("%s: i2c_address=0x%02x, flags=0x%02x, adapter=%p smbus_read=0x%x\n", __FUNCTION__,
			accel_info.i2c.addr, accel_info.i2c.flags, 
			accel_info.i2c.adapter, i2c_smbus_read_byte_data (new, ACCEL_I2C_PORT_WHO_AM_I));

	result = accel_i2c_who_am_i ();
	printk ("%s: who_am_i() returned 0x%x\n", __FUNCTION__, result);

	if ((u8)result != ACCEL_I_AM_THE_ONE) {
		fprintk ("I2C device with address 0x%x is not KXPS5", address);
		err = -ENODEV;		
		goto exit;
	}else {
		vprintk ("I2C device with address 0x%x is KXPS5 accelerometer", address);
	}

	//INIT_WORK (&accel_info.wq, accel_irq_bottom_half);

	/* turn on basic configuration just in case: Power off, DATA_READY interrupt on INT2, BDU on */
	accel_i2c_init ();

	/* configure GPIO 108 */	
/*	if (gpio_request (ACCEL_GPIO_ACL_INT2, MODULE_NAME ":ACL_INT2") == 0) {
		if (gpio_direction_input (ACCEL_GPIO_ACL_INT2) == 0) {
			accel_info.irq = gpio_to_irq (ACCEL_GPIO_ACL_INT2);

			vprintk ("GPIO %d configured as an interrupt line, irq %d", 
				ACCEL_GPIO_ACL_INT2, accel_info.irq);
		}else {
			fprintk ("error configuring gpio %d", ACCEL_GPIO_ACL_INT2);
			gpio_free (ACCEL_GPIO_ACL_INT2);
		}
	}else {
		fprintk ("failed to gpio_request %d", ACCEL_GPIO_ACL_INT2);
	}

	if (accel_info.irq > 0) {
		result = request_irq (accel_info.irq, accel_irq_handler,
					IRQF_TRIGGER_RISING, MODULE_NAME, &accel_info);
		if (result) {
			fprintk ("can't access irq %d", accel_info.irq);
			accel_info.irq = -1;
		}else {
			vprintk ("INT_2 irq = %d", accel_info.irq);
		}
	}
*/
	strlcpy(new->name, MODULE_NAME, I2C_NAME_SIZE);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client (new))) {
		fprintk ("failed to attach client");
		goto exit_irqs;
	}

	if (accel_param_input) {
		struct input_dev *input;
		
		/* take care of input device here */
		idev = input_allocate_polled_device ();

		if (! idev) {
			fprintk ("failed to allocate input polled device structure");
			err = -ENOMEM;
			goto exit_detach;
		}

		idev->poll = accel_input_poll;
		idev->poll_interval = ACCEL_POLL_INTERVAL;

		input = idev->input;		

		input->name = "Accelerometer";
		input->phys = "accelerometer";
		input->id.bustype = BUS_I2C;
		input->id.vendor  = 0;
		input->id.product = ACCEL_I_AM_THE_ONE;
		input->dev.parent = &adapter->dev;

		set_bit (EV_ABS, input->evbit);

		/* events to report absolut coordinates */
		input_set_abs_params (input, ABS_X, -2000, 2000, ACCEL_INPUT_FUZZ, ACCEL_INPUT_FLAT);
		input_set_abs_params (input, ABS_Y, -2000, 2000, ACCEL_INPUT_FUZZ, ACCEL_INPUT_FLAT);
		input_set_abs_params (input, ABS_Z, -2000, 2000, ACCEL_INPUT_FUZZ, ACCEL_INPUT_FLAT);

		if ((err = input_register_polled_device (idev))) {
			fprintk ("failed to register input polled device");
			goto exit_input;
		}

		printk ("%s: polled input device created (%d)", __FUNCTION__, err);

		accel_info.idev = idev;
		//if (accel_param_power == 0)
		//	accel_param_power = PWR_ON;
	}

//	INIT_DELAYED_WORK (&accel_info.dw, accel_irq_bottom_half);
//	accel_info.myworkqueue = create_workqueue (MODULE_NAME);
//	queue_delayed_work (accel_info.myworkqueue, &accel_info.dw, msecs_to_jiffies(ACCEL_POLL_INTERVAL)); 

	/* create char device if requested */
	if (accel_param_major) {
		dev = MKDEV(accel_param_major, accel_param_minor);
		err = register_chrdev_region(dev, accel_param_nr_devs, "accel_i2c");
	} else {
		err = alloc_chrdev_region(&dev, accel_param_minor,
					accel_param_nr_devs, "accel_i2c");
		accel_param_major = MAJOR(dev);
	}

	if (err < 0 ) {
		fprintk("can't get major %d", accel_param_major);
		goto exit_region;
	}

	vprintk ("assigned major %d", accel_param_major);

	if ((cdev = kzalloc (sizeof(struct cdev), GFP_KERNEL)) == NULL) {
		err = -ENOMEM;
		goto exit_region;
	}

	vprintk ("cdev structure allocated at 0x%x", (unsigned int)cdev);

	cdev_init (cdev, &accel_fops);
	cdev->owner = THIS_MODULE;
	cdev->ops = &accel_fops;

	vprintk ("cdev structure initialized");

	err = cdev_add (cdev, dev, 1);
	if (err) {
		fprintk("error %d adding character device", err);
		goto exit_cdev_kfree;
	}

	vprintk ("character device added");
	accel_info.cdev = cdev;

	/* Turn power on if forced to do so */
	if (accel_param_power) {
		/* aplly power settings */
		accel_i2c_set_power_mode (accel_param_power);
	}

	/* Register sysfs hooks */
	if ((err = sysfs_create_group (&new->dev.kobj, &accel_defattr_group))) {
		fprintk ("failed to create a sysfs group");
		goto exit_input;
	}

	vprintk ("driver installed successfully");

	return SUCCESS;
	
exit_cdev_kfree:
	kfree (cdev);
exit_region:
	unregister_chrdev_region(MKDEV(accel_param_major, accel_param_minor), accel_param_nr_devs);
exit_input:
	if (accel_info.idev) {
		input_free_polled_device (accel_info.idev);
		accel_info.idev = NULL;
	}
exit_detach:
	i2c_detach_client (new);
exit_irqs:
	free_irq (accel_info.irq, &accel_info);
	gpio_free (ACCEL_GPIO_ACL_INT2);
exit:
	return err;
}

/**
 * Attach_Adapter implementation
 * @param i2c I2C adapter
 * @return 0 in success, or negative error code
 */
static int accel_attach_adapter(struct i2c_adapter *i2c)
{
	return i2c_probe (i2c, &addr_data, accel_detect);
}

/**
 * Detach_Client implementation
 * @param i2c I2C client
 * @return 0 in success, or negative error code
 */
static int accel_detach_client(struct i2c_client *i2c)
{
	int err;

	/* complete all scheduled work */
	flush_scheduled_work ();

	/* remove irq handler */
	if (accel_info.irq > 0)	free_irq (accel_info.irq, &accel_info);

	/* release GPIOs */
	gpio_free (ACCEL_GPIO_ACL_INT2);

	/* power down accelerometer */
	accel_i2c_set_power_mode (PWR_OFF);

	sysfs_remove_group (&i2c->dev.kobj, &accel_defattr_group);

	if (accel_info.idev) {
		input_unregister_polled_device (accel_info.idev);
		input_free_polled_device (accel_info.idev);
	}

	if (accel_info.cdev) {
		cdev_del (accel_info.cdev);
		kfree (accel_info.cdev);
		unregister_chrdev_region(MKDEV(accel_param_major, accel_param_minor),
					       			 accel_param_nr_devs);
	}

	if ((err = i2c_detach_client (i2c)))
		return err;

	vprintk ("Driver exited");

	return 0;
}


/**
 * Init implementation
 * @return 0 in success, or negative error code
 */
static int __init accel_init(void)
{
	/* force driver to load when I2C bus is up */
	force_accel[0] = ANY_I2C_BUS;
	force_accel[1] = ACCEL_I2C_ADDRESS;
	force_accel[2] = I2C_CLIENT_END;

	return i2c_add_driver (&accel_i2c_driver);
}


/**
 * Exit implementation
 * @return 0
 */
static void __exit accel_exit(void)
{
	i2c_del_driver (&accel_i2c_driver);
}

module_init(accel_init);
module_exit(accel_exit);

