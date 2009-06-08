/*
 * Copyright (c) 2008-2009, Motorola, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/lis331dlh.h>

#define NAME "lis331dlh"

/** Maximum polled-device-reported g value */
#define G_MAX (LIS331DLH_G_8G * 1000)

#define SHIFT_ADJ_2G 4
#define SHIFT_ADJ_4G 3
#define SHIFT_ADJ_8G 2

#define AXISDATA_REG 0x28
#define NUM_AXES 3
#define BYTES_PER_AXIS 2
#define AXISDATA_BYTES (NUM_AXES*BYTES_PER_AXIS)
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define INDEX_XL 0 /* X-axis low byte entry index */
#define INDEX_XH 1 /* X-axis high byte entry index */
#define INDEX_YL 2 /* Y-axis low byte entry index */
#define INDEX_YH 3 /* Y-axis high byte entry index */
#define INDEX_ZL 4 /* Z-axis low byte entry index */
#define INDEX_ZH 5 /* Z-axis high byte entry index */


#define WHOAMI_REG 0x0f
#define WHOAMI_VAL 0x32

#define CTRL_REG1  0x20 /* power control reg */
/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define PM_OFF		0x00
#define PM_NORMAL	0x20
#define ODR_HALF	0x40 /* 0.5Hz output data rate */
#define ODR_1		0x60 /* 1Hz output data rate */
#define ODR_2		0x80 /* 2Hz output data rate */
#define ODR_5		0xa0 /* 5Hz output data rate */
#define ODR_10		0xc0 /* 10Hz output data rate */
#define ENABLE_ALL_AXES	0x07
#define ODRHALF	0x40 /* 0.5Hz output data rate */
#define ODR1	0x60 /* 1Hz output data rate */
#define ODR2	0x80 /* 2Hz output data rate */
#define ODR5	0xA0 /* 5Hz output data rate */
#define ODR10	0xC0 /* 10Hz output data rate */
#define ODR50	0x00 /* 50Hz output data rate */
#define ODR100	0x08 /* 100Hz output data rate */
#define ODR400	0x10 /* 400Hz output data rate */
#define ODR1000	0x18 /* 1000Hz output data rate */


#define CTRL_REG2 0x21 /* power control reg */
#define CTRL_REG3 0x22 /* power control reg */
#define CTRL_REG4 0x23/* interrupt control reg */

#define NUM_CTRL_REGS 5

struct {
	unsigned int cutoff;
	unsigned char mask;
} odr_table[] = {
	{3,	PM_NORMAL | ODR1000},
	{10,	PM_NORMAL | ODR400},
	{20,	PM_NORMAL | ODR100},
	{100,	PM_NORMAL | ODR50},
	{200,	ODR1000 | ODR10},
	{500,	ODR1000 | ODR5},
	{1000,	ODR1000 | ODR2},
	{2000,	ODR1000 | ODR1},
	{0,	ODR1000 | ODRHALF},
};


#define MAX_RETRIES 5
#define RETRY_DELAY 10
/* Auto-increment bit to set in I2C message addresses */
#define AUTOINCREMENT 0x80

struct lis331dlh_info {
	struct lis331dlh_platform_data *pdata;
	/* The polled device interface */
	struct input_polled_dev *polled_dev;
	/** Local I2C client */
	struct i2c_client *client;
	unsigned char shift_adj;
	unsigned char enabled;
	unsigned char resume_state[NUM_CTRL_REGS];
	int (*power_on)(void);
	int (*power_off)(void);
};

static struct lis331dlh_info info;

int lis331dlh_hw_write(unsigned int dest_addr, unsigned int num_bytes,
	unsigned char *src_buffer)
{
	unsigned char writebuffer[16];
	int bytes;
	int i = 0;

	if ((num_bytes + 1 > 16) || (src_buffer == NULL) ||
	    info.client == NULL) {
		dev_err(&info.client->dev, "invalid write parameter(s)\n");
		return -EINVAL;
	}

	writebuffer[0] = (AUTOINCREMENT | dest_addr);
	memcpy(&writebuffer[1], src_buffer, num_bytes);
	do {
		bytes = i2c_master_send(info.client, writebuffer,
				1 + num_bytes);
		if (bytes != (num_bytes + 1)) {
			dev_err(&info.client->dev, "write%i failed: %d\n", i,
				bytes);
			msleep(RETRY_DELAY);
		}
	} while ((bytes != (num_bytes + 1)) && ((++i) < MAX_RETRIES));

	if (bytes == (1 + num_bytes))
		return 0;
	return bytes;
}

int lis331dlh_update_g_range(struct lis331dlh_info *info,
			     unsigned char new_g_range)
{
	int ret;
	unsigned char shift;

	switch (new_g_range) {
	case LIS331DLH_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case LIS331DLH_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case LIS331DLH_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		return -EINVAL;
	}

	/* If device is currently enabled, we need to write new configuration
	 *  out to it */
	if (info->enabled) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		ret = lis331dlh_hw_write(CTRL_REG4, 1, &new_g_range);
		if (ret)
			return ret;
	} else {
		/* Otherwise, update the resume state value */
		info->resume_state[3] = new_g_range;
	}
	info->shift_adj = shift;

	return 0;
}

int lis331dlh_update_odr(struct lis331dlh_info *info, int poll_interval)
{
	unsigned char config;
	int i;

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		config = odr_table[i].mask;
		if (poll_interval < odr_table[i].cutoff)
			break;
	}

	config |= ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (info->enabled)
		return lis331dlh_hw_write(CTRL_REG1, 1, &config);
	else
		info->resume_state[0] = config;
	return 0;
}

int lis331dlh_hw_read(unsigned int src_addr, unsigned int num_bytes,
		      unsigned char *dest_buffer)
{
	int bytes;
	int i = 0;

	if ((num_bytes <= 0) || (dest_buffer == NULL) || info.client == NULL) {
		dev_err(&info.client->dev, "invalid read parameter(s)\n");
		return -EINVAL;
	}

	dest_buffer[0] = (AUTOINCREMENT | src_addr);

	do {
		bytes = i2c_master_send(info.client, dest_buffer, 1);
		if (bytes == 1) {
			bytes = i2c_master_recv(info.client,
						dest_buffer, num_bytes);
		}
		if (bytes != num_bytes) {
			dev_err(&info.client->dev, "read%i failed: %d\n", i,
				bytes);
			msleep(RETRY_DELAY);
		}
	} while ((bytes != num_bytes) && ((++i) < MAX_RETRIES));

	if (bytes == num_bytes)
		return 0;

	return bytes;
}

static int lis331dlh_get_acceleration_data(struct lis331dlh_info *info,
					   int *xyz)
{
	int ret;
	/** Data bytes from hardware xL, xH, yL, yH, zL, zH */
	unsigned char acc_data[AXISDATA_BYTES] = {0};
	/** X,Y,Z hardware data */
	int hw_d[NUM_AXES] = {0};

	ret = lis331dlh_hw_read((unsigned int)AXISDATA_REG, AXISDATA_BYTES,
				acc_data);
	if (ret) {
		return ret;
	}

	hw_d[AXIS_X] = (s16)(((acc_data[INDEX_XH]) << 8) | acc_data[INDEX_XL]);
	hw_d[AXIS_Y] = (s16)(((acc_data[INDEX_YH]) << 8) | acc_data[INDEX_YL]);
	hw_d[AXIS_Z] = (s16)((acc_data[INDEX_ZH]) << 8) | acc_data[INDEX_ZL];

	hw_d[AXIS_X] >>= info->shift_adj;
	hw_d[AXIS_Y] >>= info->shift_adj;
	hw_d[AXIS_Z] >>= info->shift_adj;

	xyz[AXIS_X] = ((info->pdata->negate_x) ?
		       (-hw_d[info->pdata->axis_map_x])
		       : (hw_d[info->pdata->axis_map_x]));
	xyz[AXIS_Y] = ((info->pdata->negate_y) ?
		       (-hw_d[info->pdata->axis_map_y])
		       : (hw_d[info->pdata->axis_map_y]));
	xyz[AXIS_Z] = ((info->pdata->negate_z) ?
		       (-hw_d[info->pdata->axis_map_z])
		       : (hw_d[info->pdata->axis_map_z]));

	return ret;
}

void lis331dlh_poll_handler(struct input_polled_dev *pd)
{
	struct lis331dlh_info *info = pd->private;
	int xyz[NUM_AXES] = {0};
	int axis_x, axis_y, axis_z;

	if (!info->enabled || (lis331dlh_get_acceleration_data(info, xyz)))
		return;

	axis_x = xyz[AXIS_X];
	axis_y = xyz[AXIS_Y];
	axis_z = xyz[AXIS_Z];

	if (info->polled_dev == NULL)
		return;

	input_report_abs(info->polled_dev->input, ABS_X, axis_x);
	input_report_abs(info->polled_dev->input, ABS_Y, axis_y);
	input_report_abs(info->polled_dev->input, ABS_Z, axis_z);
	input_sync(info->polled_dev->input);
}

static int lis331dlh_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &info;
	return 0;
}

static int lis331dlh_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	unsigned char pd = PM_OFF;
	struct lis331dlh_info *info = i2c_get_clientdata(client);

	if (info->enabled == 0) {
		return 0;
	}

	ret = lis331dlh_hw_read(CTRL_REG1,
				ARRAY_SIZE(info->resume_state),
				info->resume_state);

	if (ret) {
		dev_err(&info->client->dev, "resume capture failed: %d\n", ret);
		return ret;
	}

	ret = lis331dlh_hw_write(CTRL_REG4, 1, &pd);

	if (ret) {
		dev_err(&info->client->dev, "power down failed: %d\n", ret);
		return ret;
	}

	if (info->power_off)
		ret = info->power_off();

	info->enabled = 0;
	return ret;
}

static int lis331dlh_resume(struct i2c_client *client)
{
	struct lis331dlh_info *info = i2c_get_clientdata(client);
	int ret;

	if (info->enabled)
		return 0;

	if (info->power_on) {
		ret = info->power_on();
		if (ret) {
			dev_err(&client->dev, "power on failed");
			return ret;
		}
	}

	ret = lis331dlh_hw_write(CTRL_REG1,
				 ARRAY_SIZE(info->resume_state),
				 info->resume_state);

	if (ret) {
		dev_err(&client->dev, "resume failed\n");
		if (info->power_off)
			info->power_off();
		return ret;
	}
	info->enabled = 1;
	return 0;
}

static int lis331dlh_ioctl(struct inode *inode, struct file *filp,
			   unsigned int cmd, unsigned long arg)
{
	struct lis331dlh_info *info = filp->private_data;
	int ret = 0;

	switch (cmd) {
	case LIS331DLH_IOCTL_SET_POLLINTERVAL:
		if (arg < info->pdata->min_interval) {
			arg = info->pdata->min_interval;
			dev_info(&info->client->dev,
				 "minimum poll interval enforced\n");
		}
		ret = lis331dlh_update_odr(info, arg);
		break;
	case LIS331DLH_IOCTL_TRIGGER_POLLHANDLER:
		lis331dlh_poll_handler(info->polled_dev);
		break;
	case LIS331DLH_IOCTL_SET_ENABLE:
	{
		pm_message_t msg = {.event = 0,};

		if (arg == 0)
			ret = lis331dlh_suspend(info->client, msg);
		else if (arg == 1)
			ret = lis331dlh_resume(info->client);
		else
			ret = -EINVAL;
	}
		break;
	case LIS331DLH_IOCTL_GET_ENABLE:
		ret = info->enabled;
		break;
	case LIS331DLH_IOCTL_SET_G_RANGE:
		ret = lis331dlh_update_g_range(info, arg);
		break;
	default:
		dev_err(&info->client->dev, "invalid ioctl supplied\n");
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static int lis331dlh_check_pdata(struct lis331dlh_info *info)
{
	if (info->pdata == NULL) {
		dev_err(&info->client->dev, "missing platform data\n");
		return -EINVAL;
	}

	if (info->pdata->axis_map_x >= NUM_AXES ||
	    info->pdata->axis_map_y >= NUM_AXES ||
	    info->pdata->axis_map_z >= NUM_AXES) {
		dev_err(&info->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			info->pdata->axis_map_x, info->pdata->axis_map_y,
			info->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (info->pdata->negate_x > 1 || info->pdata->negate_x > 1 ||
	    info->pdata->negate_x > 1) {
		dev_err(&info->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			info->pdata->negate_x, info->pdata->negate_y,
			info->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (info->pdata->interval < info->pdata->min_interval) {
		dev_err(&info->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

/** Driver file operations interface */
static const struct file_operations lis331dlh_fops = {
	.owner	= THIS_MODULE,
	.open	= lis331dlh_open,
	.ioctl	= lis331dlh_ioctl,
};

/** Driver miscellaneous device interface */
static struct miscdevice lis331slh_miscdev = {
	.fops  = &lis331dlh_fops,
	.minor = MISC_DYNAMIC_MINOR,
	.name  = NAME,
};

static int lis331dlh_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lis331dlh_platform_data *pdata = client->dev.platform_data;
	int ret;
	unsigned char identity;

	info.client = client;
	i2c_set_clientdata(info.client, &info);
	info.enabled = 1;

	dev_info(&info.client->dev, "probing accelerometer\n");

	info.pdata = pdata;
	ret = lis331dlh_check_pdata(&info);
	if (ret) {
		dev_err(&info.client->dev, "failed reading platform data\n");
		ret = -ENODEV;
		goto error;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&info.client->dev, "needs I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto error;
	}

	info.power_on = pdata->power_on;
	info.power_off = pdata->power_off;

	if (pdata->init)
		pdata->init();

	if (info.power_on)
		info.power_on();

	ret = lis331dlh_hw_read(WHOAMI_REG, 1, &identity);
	if (ret || identity != WHOAMI_VAL) {
		ret = -ENXIO;
		pr_err("%s: WHOAMI failed: %d\n", NAME, ret);
		goto error;
	}

	if (info.power_off)
		info.power_off();

	memset(info.resume_state, 0, ARRAY_SIZE(info.resume_state));
	ret = lis331dlh_update_g_range(&info, pdata->g_range);
	if (ret) {
		pr_err("%s: update_g_range failed\n", NAME);
		goto error;
	}

	/* Initialize the power resume state value via update_odr - driver is
	 * currently disabled so this function will write into resume state */
	ret = lis331dlh_update_odr(&info, pdata->interval);
	if (ret) {
		pr_err("%s: update_odr failed\n", NAME);
		goto error;
	}

	info.polled_dev = input_allocate_polled_device();
	if (!info.polled_dev) {
		ret = -ENOMEM;
		pr_err("%s: failed to allocate polled device\n", NAME);
		goto error;
	}
	info.polled_dev->private = &info;
	info.polled_dev->poll = lis331dlh_poll_handler;
	info.polled_dev->poll_interval = pdata->interval; /* ms */
	info.polled_dev->input->name = NAME;
	info.polled_dev->input->phys = NAME;
	set_bit(EV_ABS, info.polled_dev->input->evbit);
	input_set_abs_params(info.polled_dev->input, ABS_X,
			     -G_MAX, G_MAX, info.pdata->fuzz,
			     info.pdata->flat);
	input_set_abs_params(info.polled_dev->input, ABS_Y,
			     -G_MAX, G_MAX, info.pdata->fuzz,
			     info.pdata->flat);
	input_set_abs_params(info.polled_dev->input, ABS_Z,
			     -G_MAX, G_MAX, info.pdata->fuzz,
			     info.pdata->flat);

	ret = input_register_polled_device(info.polled_dev);
	if (ret) {
		pr_err("%s: failed to register polled device\n", NAME);
		goto error_input_register_error;
	}

	ret = misc_register(&lis331slh_miscdev);
	if (ret) {
		pr_err("%s: failed to register misc device\n", NAME);
		goto error_misc_register_error;
	}

	dev_info(&info.client->dev, "initialization complete\n");
	return 0;

error_misc_register_error:
	input_unregister_polled_device(info.polled_dev);
error_input_register_error:
	input_free_polled_device(info.polled_dev);
error:
	if (pdata->exit)
		pdata->exit();
	return ret;
}

static int lis331dlh_remove(struct i2c_client *client)
{
	struct lis331dlh_platform_data *pdata = client->dev.platform_data;
	pm_message_t msg = {.event = 0,};

	misc_deregister(&lis331slh_miscdev);
	if (info.polled_dev) {
		input_unregister_polled_device(info.polled_dev);
		input_free_polled_device(info.polled_dev);
	}

	lis331dlh_suspend(info.client, msg);
	info.client = NULL;

	if (pdata->exit)
		pdata->exit();

	return 0;
}


static const struct i2c_device_id lis331dlh_id[] = {
	{ NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, lis331dlh_id);

static struct i2c_driver lis331dlh_i2c_driver = {
	.driver = {
		.name  = NAME,
		.owner = THIS_MODULE,
	},
	.probe = lis331dlh_probe,
	.remove = __devexit_p(lis331dlh_remove),
	.id_table = lis331dlh_id,
};


static int  __init lis331dlh_init(void)
{
	return i2c_add_driver(&lis331dlh_i2c_driver);
}

static void __exit lis331dlh_exit(void)
{
	i2c_del_driver(&lis331dlh_i2c_driver);
}

module_init(lis331dlh_init);
module_exit(lis331dlh_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("STMicro LIS331DLH driver");
MODULE_LICENSE("GPL");

