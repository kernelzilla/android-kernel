/*
 * drivers/media/video/omap/hplens.c
 *
 * HP Generic Driver : Driver implementation for generic lens module.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>

#include "oldomap34xxcam.h"
#include "hplens.h"

#define DRIVER_NAME  "hplens"

static int hplens_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int __exit hplens_remove(struct i2c_client *client);

struct hplens_device {
	const struct hplens_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	int opened;
	int state;
	int power_state;
};

static const struct i2c_device_id hplens_id[] = {
	{ HPLENS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hplens_id);

static struct i2c_driver hplens_i2c_driver = {
	.driver = {
		.name = HPLENS_NAME,
		.owner = THIS_MODULE,
	},
	.probe = hplens_probe,
	.remove = __exit_p(hplens_remove),
	.id_table = hplens_id,
};

static struct hplens_device hplens = {
	.state = LENS_NOT_DETECTED,
	.power_state = 0,
};

static struct vcontrol {
	struct v4l2_queryctrl qc;
} video_control[] = {
   {
		{
		.id = V4L2_CID_HPLENS_CMD_READ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Read",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		},
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_WRITE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Write",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_READ_PAGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Read Page",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_WRITE_PAGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Write Page",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_CAL_READ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Cal Read",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   }
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/**
 * hplens_reg_read - Reads a value from a register in an I2C driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Pointer to u16 for returning value of register to read.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
int hplens_reg_read(u8 dev_addr, u8 *value, u16 len)
{
	struct hplens_device *lens = &hplens;
	struct i2c_client *client = lens->i2c_client;
	int err;
	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	client->addr = dev_addr;  /* set slave address */

	msg->addr = client->addr;
	msg->flags = I2C_M_RD;
	msg->len = len;
	msg->buf = value;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
		return 0;

	return err;
}
EXPORT_SYMBOL(hplens_reg_read);

/**
 * hplens_reg_write - Writes a value to a register in LENS Coil driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Value of register to write.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
int hplens_reg_write(u8 dev_addr, u8 *write_buf, u16 len)
{
	struct hplens_device *lens = &hplens;
	struct i2c_client *client = lens->i2c_client;
	int err;
	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	client->addr = dev_addr;  /* set slave address */

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = len;
	msg->buf = write_buf;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
		return 0;

	return err;
}
EXPORT_SYMBOL(hplens_reg_write);

/**
 * hplens_ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int hplens_ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct hplens_device *lens = s->priv;

	if (lens->pdata->priv_data_set)
		return lens->pdata->priv_data_set(p);

	return -1;
}

 /**
 * hplens_ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int hplens_ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct hplens_device *lens = s->priv;

	if (lens->pdata->power_set)
		lens->pdata->power_set(on);

	/* May want to replace with api_ReportMotorType(); */

	lens->power_state = on;
	return 0;
}

/**
 * hplens_ioctl_queryctrl - V4L2 lens interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int hplens_ioctl_queryctrl(struct v4l2_int_device *s,
			struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;

	return 0;
}

/**
 * hplens_ioctl_s_ctrl - V4L2 lens interface handler for  VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the imx046sensor_video_control[] array).
 * Otherwise, * returns -EINVAL if the control is not supported.
 */
static int hplens_ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int ret = -EINVAL;
	struct hplens_reg reg;
	struct hplens_eeprom *eeprom = NULL;
	u8 write_buffer[16];
	u8 fdb = 0;
	int idx;

	if (find_vctrl(vc->id) < 0)
		return -EINVAL;

	switch (vc->id) {
	case V4L2_CID_HPLENS_CMD_READ: {
		ret = copy_from_user(&reg, (void *)vc->value,  \
			sizeof(struct hplens_reg));
		if (ret == 0) {
			if (reg.addr[0] != 0xff) {
				/* valid register address */
				/* write the register address to read */
				ret = hplens_reg_write(reg.dev_addr, \
					reg.addr, reg.len_addr);
			}
			/* Read the register */
			ret = hplens_reg_read(reg.dev_addr, \
				reg.data, reg.len_data);
			if (ret == 0) {
				ret = copy_to_user((void *)vc->value, &reg, \
					sizeof(struct hplens_reg));
			}
		}
	}
	break;
	case V4L2_CID_HPLENS_CMD_WRITE: {
		ret = copy_from_user(&reg, (void *)vc->value,  \
			sizeof(struct hplens_reg));
		if (ret == 0) {
			if (reg.addr[0] != 0xff) { /* valid register address */
				while (fdb < reg.len_addr) {
					/* write register address in buffer */
					write_buffer[fdb] = reg.addr[fdb];
					fdb++;
				}
			}

			for (idx = fdb; idx <= reg.len_data; idx++)
				write_buffer[idx] = reg.data[idx-fdb];

			ret = hplens_reg_write(reg.dev_addr, \
				write_buffer, reg.len_data + fdb);
		}
	}
	break;
	case V4L2_CID_HPLENS_CMD_CAL_READ: {

		/* Using dynamic memory. */
		eeprom = kmalloc(sizeof(struct hplens_eeprom), GFP_KERNEL);
		if (eeprom == NULL)
			return -EINVAL;

		ret = copy_from_user(eeprom, (void *)vc->value,  \
			sizeof(struct hplens_eeprom));
		if (ret == 0) {
			if (eeprom->addr[0] != 0xff) {
				/* valid register address */
				/* write the register address to read */
				ret = hplens_reg_write(eeprom->dev_addr, \
					eeprom->addr, eeprom->len_addr);
			}

			/* Read the register */
			ret = hplens_reg_read(eeprom->dev_addr, \
				eeprom->data, eeprom->len_data);
			if (ret == 0)
				ret = copy_to_user((void *)vc->value, eeprom, \
					sizeof(struct hplens_eeprom));
		}

		/* clean up. */
		if (eeprom != NULL)
			kfree(eeprom);
	}
	break;
	case V4L2_CID_HPLENS_CMD_READ_PAGE:
		/* TODO: Implement */
	break;
	case V4L2_CID_HPLENS_CMD_WRITE_PAGE:
		/* TODO: Implement */
	break;
	}

	return ret;
}

static struct v4l2_int_ioctl_desc hplens_ioctl_desc[] = {
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_g_priv },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_queryctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_s_ctrl },
};

static struct v4l2_int_slave hplens_slave = {
	.ioctls = hplens_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(hplens_ioctl_desc),
};

static struct v4l2_int_device hplens_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.priv = &hplens,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &hplens_slave,
	},
};

/**
 * lens_probe - Probes the driver for valid I2C attachment.
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -EBUSY if unable to get client attached data.
 **/
static int hplens_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hplens_device *lens = &hplens;
	int err;

	dev_info(&client->dev, "lens probe called....\n");

	if (i2c_get_clientdata(client)) {
		printk(KERN_ERR " DTA BUSY %s\n", client->name);
		return -EBUSY;
	}

	lens->pdata = client->dev.platform_data;

	if (!lens->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	lens->v4l2_int_device = &hplens_int_device;

	lens->i2c_client = client;
	i2c_set_clientdata(client, lens);

	err = v4l2_int_device_register(lens->v4l2_int_device);
	if (err) {
		printk(KERN_ERR "Failed to Register " \
			DRIVER_NAME " as V4L2 device.\n");
		i2c_set_clientdata(client, NULL);
	} else {
		printk(KERN_ERR "Registered " DRIVER_NAME " as V4L2 device.\n");
	}

	return 0;
}

 /**
 * lens_remove - Routine when device its unregistered from I2C
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -ENODEV if the client isn't attached.
 **/
static int __exit hplens_remove(struct i2c_client *client)
{
	if (!client->adapter)
		return -ENODEV;

	i2c_set_clientdata(client, NULL);
	return 0;
}

/**
 * lens_init - Module initialisation.
 *
 * Returns 0 if successful, or -EINVAL if device couldn't be initialized, or
 * added as a character device.
 **/
static int __init hplens_init(void)
{
	int err = -EINVAL;

	err = i2c_add_driver(&hplens_i2c_driver);
	if (err)
		goto fail;
	return err;
fail:
	printk(KERN_ERR "Failed to register " DRIVER_NAME ".\n");
	return err;
}
late_initcall(hplens_init);

/**
 * lens_cleanup - Module cleanup.
 **/
static void __exit hplens_cleanup(void)
{
	i2c_del_driver(&hplens_i2c_driver);
}
module_exit(hplens_cleanup);

MODULE_AUTHOR("Hewlett-Packard Co.");
MODULE_DESCRIPTION("HP Generic Lens Driver");
MODULE_LICENSE("GPL");
