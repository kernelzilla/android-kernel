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

#include <linux/platform_device.h>

#include "accel-main.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("STmicro 3-axis accelerometer driver");
MODULE_AUTHOR("Konstantin Makariev <hcv867@motorola.com>");

#ifdef  CONFIG_HAS_EARLYSUSPEND
static void accel_early_suspend(struct early_suspend *handler);
static void accel_early_resume(struct early_suspend *handler);
#endif

/*
 *  Global variables definition
 */
accel_data_t accel_info;
/*
 * LIS331DLH has an ability to generate interrupt to identify portrait/landscape switch.
 * However this mechanism has limitations when all 3 axes enabled. So driver has a block
 * that does some math to determine device orientation. This implementation is even more
 * powerful, since it can detect when device is face up or down in addition to 
 * portrait/landscape recognition.
 */
int accel_orient_by_interrupt = 0;	/* when 1, tells to use interrupt to determine screen orientation change */
int accel_gpio_one = -1;
int accel_gpio_two = -1;
unsigned long accel_events_mask = 0;	/* global events mask: when it's 0 we don't look through list of private data */
/*
 * Copernicus board mounted on LaJolla P2 board had a coordinate system with
 * axises Y and Z swapped. Thus flag accel_param_invert is called to fix it.
 * We don't need it in Morrison though.
 */
int accel_param_swapped = 0;

/*
 *  Driver's parameters
 */
int accel_param_trace_verbose = 1;
int accel_param_trace_debug = 1;
int accel_param_trace_data = 0;
int accel_param_trace_orient = 1;
int accel_param_trace_irq = 0;
int accel_param_trace_poll = 0;
int accel_param_trace_bad_data = 0;

int accel_param_power = 0;
int accel_param_input = 1;


module_param_named(trace_verbose, accel_param_trace_verbose, int, 0664);
module_param_named(trace_data, accel_param_trace_data, int, 0664);
module_param_named(trace_orient, accel_param_trace_orient, int, 0664);
module_param_named(trace_debug, accel_param_trace_debug, int, 0664);
module_param_named(trace_irq, accel_param_trace_irq, int, 0664);
module_param_named(trace_poll, accel_param_trace_poll, int, 0664);
module_param_named(trace_bad_data, accel_param_trace_bad_data, int, 0664);

module_param(accel_param_power, int, S_IRUGO);
module_param(accel_param_input, int, S_IRUGO);
module_param(accel_param_swapped, int, S_IRUGO);


void accel_input_params(struct input_dev *input, unsigned char param, 
    int cnt, int limit)
{
	int i;
	for (i=0; i < cnt; i++) {
		input_set_abs_params (input, param++, -limit, limit, 
					ACCEL_INPUT_FUZZ, ACCEL_INPUT_FLAT);
	}
}


static void accel_irq_config_edge(int *irq, int gpio, int irq_num, 
    void *data_id, irqreturn_t (*handler)(int, void*))
{
	char gpio_name[32];

	snprintf (gpio_name, 31, "%s:ACL_INT%d", MODULE_NAME, irq_num);

	/* configure GPIO */	
	if (gpio_request (gpio, gpio_name) == 0) {
		/* set gpio to input direction to allow irq signals */
		if (gpio_direction_input (gpio) == 0) {
			*irq = gpio_to_irq (gpio);

			vprintk ("GPIO %d configured as an interrupt line, irq %d, current value %d", 
				gpio, *irq, gpio_get_value (gpio));
		}else {
			fprintk ("error configuring gpio %d", gpio);
			return;
		}
	}else {
		fprintk ("failed to gpio_request %d", gpio);
	}

	if (*irq > 0) {
		if (request_irq (*irq, handler, IRQF_TRIGGER_RISING, MODULE_NAME, 
            data_id)) {
			fprintk ("can't access irq %d", *irq);
			*irq = -1;
		}else {
			vprintk ("irq = %d", *irq);
		}
	}
}

#ifdef ACCEL_DATA_READY_IRQ
static void accel_irq_config_high(int *irq, int gpio, int irq_num, void *data_id, irqreturn_t (*handler)(int, void*))
{
	char gpio_name[32];

	snprintf (gpio_name, 31, "%s:ACL_INT%d", MODULE_NAME, irq_num);

	/* configure GPIO */	
	if (gpio_request (gpio, gpio_name) == 0) {
		/* set gpio to input direction to allow irq signals */
		if (gpio_direction_input (gpio) == 0) {
			*irq = gpio_to_irq (gpio);

			vprintk ("GPIO %d configured as an interrupt line, irq %d, current value %d", 
				gpio, *irq, gpio_get_value (gpio));
		}else {
			fprintk ("error configuring gpio %d", gpio);
			return;
		}
	}else {
		fprintk ("failed to gpio_request %d", gpio);
	}

	if (*irq > 0) {
		if (request_irq (*irq, handler, IRQF_TRIGGER_HIGH, MODULE_NAME, data_id)) {
			fprintk ("can't access irq %d", *irq);
			*irq = -1;
		}else {
			vprintk ("irq = %d", *irq);
		}
	}
}
#endif

int accel_configure_irqs (unsigned int enable)
{
	u8 power_mode;

	power_mode = (u8)i2c_smbus_read_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1);
	i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 0x00);
	if (enable) {
		if (accel_info.irq1 == -1)
			accel_irq_config_edge (&accel_info.irq1, accel_gpio_one, ACCEL_IRQ_INT1,
				  (void *)&accel_info.irq1_cnt, accel_irq_handler);
#ifdef ACCEL_DATA_READY_IRQ
		if (accel_info.irq2 == -1)
			accel_irq_config_high (&accel_info.irq2, accel_gpio_two, ACCEL_IRQ_INT2,
				  (void *)&accel_info.irq2_cnt, accel_irq_data_ready_handler);	
#endif
	} else {
		if (accel_info.irq1 > 0) {
			gpio_free (accel_gpio_one);
			free_irq (accel_info.irq1, (void *)&accel_info.irq1_cnt);
		}
		if (accel_info.irq2 > 0) {
			gpio_free (accel_gpio_two);
			free_irq (accel_info.irq2, (void *)&accel_info.irq2_cnt);
		}
		accel_info.irq1 = accel_info.irq2 = -1;
	}
	i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, power_mode);
    return 0;
}

static struct miscdevice accel_misc_driver = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lis331dlh",
	.fops = &accel_fops,
};

/**
 * Probe implementation
 * Inits the character device and completes the I2C driver initialization
 * @return 0 in success, or negative error code
 */
static int accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err=-ENOMEM;
	struct input_dev *idev;
    struct input_polled_dev *ipdev;
	struct platform_device *pdev;

	if (!i2c_check_functionality (client->adapter, I2C_FUNC_I2C)) {
		fprintk ("I2C function is not supported");
		return -ENOTSUPP;
	}

	memset (&accel_info, 0, sizeof (accel_data_t));
	accel_info.magic = ACCEL_MAGIC;
	accel_info.irq1 = accel_info.irq2 = -1;

	mutex_init (&accel_info.mlock);
	mutex_init (&pl);
    atomic_set (&accel_info.data_ready_enabled, 0);

	accel_info.i2c = client;
	i2c_set_clientdata (client, &accel_info);

	if (accel_i2c_who_am_i () != ACCEL_I_AM_THE_ONE) {
		fprintk ("I2C device with address 0x%x is not LIS331DLH", client->addr);
		err = -ENODEV;		
		goto exit;
	}else {
		vprintk ("I2C device with address 0x%x is LIS331DLH accelerometer", client->addr);
	}

	INIT_WORK (&accel_info.wq1, accel_irq_bottom_half);
#ifdef ACCEL_DATA_READY_IRQ
	INIT_WORK (&accel_info.wq2, accel_irq_data_ready_bottom_half);
#endif

//	sema_init (&accel_info.sema1, 1);
//	sema_init (&accel_info.sema2, 1);

	pdev = (struct platform_device *)client->dev.platform_data;
	/* 
	 * IRQ field contain information about swapped axes
	 */
	accel_param_swapped = pdev->id;
	fprintk ("swapped axes parameter 0x%02x", accel_param_swapped);

	if (pdev && pdev->num_resources && ! strcmp(pdev->name, "stmicro")) {
		struct resource *resource;
		
		vprintk ("platform data for '%s', num_resources %d", 
            pdev->name, pdev->num_resources);

		resource = platform_get_resource (pdev, IORESOURCE_IRQ, 0);
		if (resource) {
			accel_gpio_one = resource->start;
			accel_gpio_two = resource->end;

			accel_irq_config_edge (&accel_info.irq1, accel_gpio_one, 
                ACCEL_IRQ_INT1,
			    (void *)&accel_info.irq1_cnt, accel_irq_handler);
#ifdef ACCEL_DATA_READY_IRQ
            accel_irq_config_high (&accel_info.irq2, accel_gpio_two, 
                ACCEL_IRQ_INT2,
                (void *)&accel_info.irq2_cnt, 
                accel_irq_data_ready_handler);	
#endif
		} else {
			fprintk ("unable to obtain GPIO information; no IRQ support");
        }
	} else {
		fprintk ("platform data has no resources declared; no IRQ support");
    }

	if (accel_param_input) {
		/* take care of input device here */
		ipdev = input_allocate_polled_device ();

		if (! ipdev) {
			fprintk ("failed to allocate input device structure");
			err = -ENOMEM;
			goto exit_irqs;
		}
        ipdev->poll_interval = (int)-1;
        ipdev->poll = accel_poll_data_ready;
         
        idev = ipdev->input;

		idev->name = "accelerometer";
		idev->id.bustype = BUS_I2C;
		idev->id.vendor  = 0;
		idev->id.product = ACCEL_I_AM_THE_ONE;
        accel_info.idev = idev;
		accel_info.ipdev = ipdev;

		set_bit (EV_ABS, idev->evbit);
		set_bit (EV_SW,  idev->evbit);

		set_bit (6, idev->swbit);
		input_report_switch (idev, 6, TOGGLE_OFF);

		accel_input_params (idev, ABS_X, 3, ACCEL_2G_MAX);
		accel_input_params (idev, ABS_RX, 3, ACCEL_2G_MAX);
		accel_input_params (idev, ABS_TILT_X, 2, ACCEL_2G_MAX);

		if ((err = input_register_polled_device (ipdev))) {
			fprintk ("failed to register input device");
			goto exit_input;
		}
	}
    fprintk ("registered input device accelerometer\n");

    atomic_set (&accel_info.mode, 0);
    atomic_set (&accel_info.odr, 0);
    atomic_set (&accel_info.g_range, 1); // 4G
	accel_i2c_set_config (0, 0, 0, -1);

	/* create misc device and /dev/accelerometer file */
	if (misc_register (&accel_misc_driver)) {
		fprintk("can't register misc. device");
		goto exit_input;
	}
    fprintk ("registered misc device\n");

	/* Turn power on if forced to do so */
	if (accel_param_power) {
		/*enable all axises */
		accel_i2c_enable_axis (AXIS_XYZ);
		/* aplly power settings */
		accel_i2c_set_power_mode (accel_param_power);
	}

	/* Register sysfs hooks */
	if ((err = sysfs_create_group (&client->dev.kobj, &accel_defattr_group))) {
		fprintk ("failed to create a sysfs group");
		goto exit_input;
	}
    fprintk ("created sysfs group\n");

#ifdef  CONFIG_HAS_EARLYSUSPEND
	accel_info.early_suspend.suspend = accel_early_suspend;
	accel_info.early_suspend.resume = accel_early_resume;
	register_early_suspend (&(accel_info.early_suspend));
#endif

	printk (KERN_INFO "%s: driver probed successfully\n", __func__);

	return SUCCESS;

exit_input:
	if (accel_info.ipdev) {
		input_free_polled_device (accel_info.ipdev);
		accel_info.ipdev = NULL;
	}
exit_irqs:
	if (accel_info.irq1 > 0) {
		gpio_free (accel_gpio_one);
		free_irq (accel_info.irq1, (void *)&accel_info.irq1_cnt);
	}
#ifdef ACCEL_DATA_READY_IRQ
	if (accel_info.irq2 > 0) {
		gpio_free (accel_gpio_two);
		free_irq (accel_info.irq2, (void *)&accel_info.irq2_cnt);
	}
#endif
exit:
	return err;
}


/**
 * Remove implementation
 * @param i2c I2C client
 * @return 0 in success, or negative error code
 */
static int accel_remove(struct i2c_client *i2c)
{
	u8 buf[2] = {ACCEL_I2C_PORT_REG1, 0x0};

	/* complete all scheduled work */
	flush_scheduled_work ();

	/* remove irq handler */
	if (accel_info.irq1 > 0) {
		free_irq (accel_info.irq1, &accel_info);
		/* release GPIOs */
		gpio_free (accel_gpio_one);
	}

#ifdef ACCEL_DATA_READY_IRQ
	if (accel_info.irq2 > 0) {
		free_irq (accel_info.irq2, &accel_info);
		/* release GPIOs */
		gpio_free (accel_gpio_two);
	}
#endif

	/* power down accelerometer */
	i2c_master_send (i2c, buf, 2);

	sysfs_remove_group (&i2c->dev.kobj, &accel_defattr_group);

	if (accel_info.idev) {
		input_unregister_polled_device (accel_info.ipdev);
		input_free_polled_device (accel_info.ipdev);
	}

	misc_deregister (&accel_misc_driver);

/*	platform_device_del (accel_platform_device);
	device_remove_file (&accel_platform_device->dev, );
*/
	vprintk ("driver exited");
	return 0;
}

static union {
	u8	array[13];
	struct {
	u8	ctrl_1, ctrl_2,
		ctrl_3, ctrl_4, ctrl_5;
	u8	int1_cfg, int1_src, int1_ths, int1_dur,
		int2_cfg, int2_src, int2_ths, int2_dur;
	}	registers;
}	accel_safe;

/**
 * Suspend implementation
 *
 * @param i2c I2C device
 * @param state PM state
 * @return 0
 */
static int accel_suspend(struct i2c_client *client, pm_message_t mesg)
{
	// cancel all pending tasks
//	down (&accel_info.sema1);		// ensures BH is not running
	cancel_work_sync (&accel_info.wq1);	// disable further qeueing
//	up (&accel_info.sema1);
//	down (&accel_info.sema2);		// ensures BH is not running

#ifdef ACCEL_DATA_READY_IRQ
    cancel_work_sync (&accel_info.wq2);
#endif
//	up (&accel_info.sema2);

	mutex_lock (&accel_info.mlock);
	//accel_i2c_read_control_registers (accel_safe.array, 13);
    accel_safe.registers.int1_cfg = accel_i2c_read_byte_data (ACCEL_I2C_PORT_INT1_CFG);
    accel_safe.registers.int1_ths = accel_i2c_read_byte_data (ACCEL_I2C_PORT_INT1_THS);
    accel_safe.registers.int1_dur = accel_i2c_read_byte_data (ACCEL_I2C_PORT_INT1_DUR);
    accel_safe.registers.int2_cfg = accel_i2c_read_byte_data (ACCEL_I2C_PORT_INT2_CFG);
    accel_safe.registers.int2_ths = accel_i2c_read_byte_data (ACCEL_I2C_PORT_INT2_THS);
    accel_safe.registers.int2_dur = accel_i2c_read_byte_data (ACCEL_I2C_PORT_INT2_DUR);

    accel_safe.registers.ctrl_4 = accel_i2c_read_byte_data (ACCEL_I2C_PORT_REG4);
    accel_safe.registers.ctrl_3 = accel_i2c_read_byte_data (ACCEL_I2C_PORT_REG3);
    accel_safe.registers.ctrl_1 = accel_i2c_read_byte_data (ACCEL_I2C_PORT_REG1);

	i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 0x00); // power off
	accel_info.suspended = 1;
	mutex_unlock (&accel_info.mlock);

	dprintk ("driver suspended");
	client=client;
	mesg=mesg;
	return (0);
}

/**
 * Resume implementation
 *
 * @param i2c I2C device
 * @return 0
 */
int accel_resume(struct i2c_client *client)
{
	mutex_lock (&accel_info.mlock);
	if (! accel_info.suspended) {
		mutex_unlock (&accel_info.mlock);	
		dprintk ("driver has been resumed already");
	} else {
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_INT1_CFG, accel_safe.registers.int1_cfg);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_INT1_THS, accel_safe.registers.int1_ths);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_INT1_DUR, accel_safe.registers.int1_dur);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_INT2_CFG, accel_safe.registers.int2_cfg);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_INT2_THS, accel_safe.registers.int2_ths);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_INT2_DUR, accel_safe.registers.int2_dur);

		accel_i2c_write_byte_data (ACCEL_I2C_PORT_REG5, 0);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_REG4, accel_safe.registers.ctrl_4);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_REG3, accel_safe.registers.ctrl_3);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_REG2, 0);
		accel_i2c_write_byte_data (ACCEL_I2C_PORT_REG1, accel_safe.registers.ctrl_1);

		accel_info.suspended = 0;
		mutex_unlock (&accel_info.mlock);

		dprintk ("driver resumed");
	}
	return(0);
}

#ifdef  CONFIG_HAS_EARLYSUSPEND
static void accel_early_suspend(struct early_suspend *handler)
{
        accel_suspend(accel_info.i2c, PMSG_SUSPEND);
	
}

static void accel_early_resume(struct early_suspend *handler)
{
        accel_resume(accel_info.i2c);
}
#endif

static const struct i2c_device_id accel_id[] = {
	{ MODULE_NAME, 0 },
	{ }
};

static struct i2c_driver accel_i2c_driver = {
	.id_table	= accel_id,
#ifndef  CONFIG_HAS_EARLYSUSPEND
	.suspend	= accel_suspend,
	.resume		= accel_resume,
#endif
	.probe		= accel_probe,
	.remove		= accel_remove,
	.driver		= {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		},
};

/**
 * Init implementation
 * @return 0 in success, or negative error code
 */
static int __devinit accel_init(void)
{
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

subsys_initcall(accel_init);
module_exit(accel_exit);

