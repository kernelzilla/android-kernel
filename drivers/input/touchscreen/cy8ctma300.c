/* Source for:
 * Cypress CY8CTMA300 Prototype touchscreen driver.
 * drivers/input/touchscreen/cy8ctma300.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 * History:
 *			(C) 2010 Cypress - Update for GPL distribution
 *			(C) 2009 Cypress - Assume maintenance ownership
 *			(C) 2009 Enea - Original prototype
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/input/cy8ctma300.h>

/* Register addresses */
#define STATUS_REG	0x01
#define DATA_REG	0x03

#define UPDATE_COORDS	0x04
#define INVALID_DATA	0xff
#define BYTES_PER_TOUCH	8
#define TOUCH_META_INFO	3
#define NUM_TOUCH_INDEX	(ts->pdata->nfingers * BYTES_PER_TOUCH)
#define FINGER_SIZE	70

#define X_INDEX		6
#define Y_INDEX		4
#define Z_INDEX		3
#define ID_INDEX	0

#define TOUCHSCREEN_TIMEOUT	(msecs_to_jiffies(10))
#define INITIAL_DELAY		(msecs_to_jiffies(25000))

struct cy8ctma300 {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct cy8ctma300_platform_data *pdata;
	u8 *touch_data;
	u8 prev_touches;
};

static inline u16 join_bytes(u8 a, u8 b)
{
	u16 ab = 0;
	ab = ab | a;
	ab = ab << 8 | b;
	return ab;
}

static s32 cy8ctma300_write_reg_u8(struct i2c_client *client, u8 reg, u8 val)
{
	s32 data;

	data = i2c_smbus_write_byte_data(client, reg, val);
	if (data < 0)
		dev_err(&client->dev, "error %d in writing reg 0x%x\n",
						 data, reg);

	return data;
}

static int cy8ctma300_read(struct i2c_client *client, u8 reg, u8 *buf, int num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = &reg;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	return i2c_transfer(client->adapter, xfer_msg, 2);
}

static void cy8ctma300_xy_worker(struct work_struct *work)
{
	struct cy8ctma300 *ts = container_of(work, struct cy8ctma300,
				 work.work);
	int rc;
	u8 pressure, i, touches, id;
	u16 x, y;

	/* read data from DATA_REG */
	rc = cy8ctma300_read(ts->client, DATA_REG, ts->touch_data,
		ts->pdata->nfingers * BYTES_PER_TOUCH + TOUCH_META_INFO);
	if (rc < 0) {
		dev_err(&ts->client->dev, "read failed\n");
		goto schedule;
	}

	touches = ts->touch_data[NUM_TOUCH_INDEX];
	if (touches == INVALID_DATA)
		goto schedule;

	/* report all touches */
	for (i = 0; i < touches; i++) {
		id = ts->touch_data[i * BYTES_PER_TOUCH + ID_INDEX];
		pressure = ts->touch_data[i * BYTES_PER_TOUCH + Z_INDEX];
		x = join_bytes(ts->touch_data[i * BYTES_PER_TOUCH + X_INDEX],
			ts->touch_data[i * BYTES_PER_TOUCH + X_INDEX + 1]);
		y = join_bytes(ts->touch_data[i * BYTES_PER_TOUCH + Y_INDEX],
			ts->touch_data[i * BYTES_PER_TOUCH + Y_INDEX + 1]);

		if (ts->pdata->swap_xy)
			swap(x, y);

		/* handle inverting coordinates */
		if (ts->pdata->invert_x)
			x = ts->pdata->res_x - x;

		if (ts->pdata->invert_y)
			y = ts->pdata->res_y - y;

		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, FINGER_SIZE);
		input_mt_sync(ts->input);
	}
	/* report releases */
	for (i = 0; i < ts->prev_touches - touches; i++) {
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input);
	}
	input_sync(ts->input);
	ts->prev_touches = touches;

schedule:
	if (ts->pdata->use_polling)
		queue_delayed_work(ts->wq, &ts->work, TOUCHSCREEN_TIMEOUT);
	else
		enable_irq(ts->client->irq);

	/* write to STATUS_REG to update coordinates*/
	rc = cy8ctma300_write_reg_u8(ts->client, STATUS_REG, UPDATE_COORDS);
	if (rc < 0) {
		dev_err(&ts->client->dev, "write failed, try once more\n");

		rc = cy8ctma300_write_reg_u8(ts->client, STATUS_REG,
				UPDATE_COORDS);
		if (rc < 0)
			dev_err(&ts->client->dev, "write failed, exiting\n");
	}
}

static irqreturn_t cy8ctma300_irq(int irq, void *dev_id)
{
	struct cy8ctma300 *ts = dev_id;

	disable_irq_nosync(irq);
	queue_delayed_work(ts->wq, &ts->work, 0);

	return IRQ_HANDLED;
}

static int cy8ctma300_init_ts(struct i2c_client *client, struct cy8ctma300 *ts)
{

	struct input_dev *input_device;
	int rc;

	input_device = input_allocate_device();
	if (!input_device)
		return -ENOMEM;

	ts->input = input_device;
	input_device->name = ts->pdata->ts_name;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;

	set_bit(EV_ABS, input_device->evbit);
	input_set_abs_params(input_device, ABS_MT_POSITION_X,
			ts->pdata->dis_min_x, ts->pdata->dis_max_x, 0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_Y,
			ts->pdata->dis_min_y, ts->pdata->dis_max_y, 0, 0);
	input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR,
			ts->pdata->min_touch, ts->pdata->max_touch, 0, 0);
	input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR,
			ts->pdata->min_width, ts->pdata->max_width, 0, 0);
	input_set_abs_params(input_device, ABS_MT_TRACKING_ID,
			ts->pdata->min_tid, ts->pdata->max_tid, 0, 0);
	/*setting dummy key to make it work for virutal keys*/
	input_set_capability(input_device, EV_KEY, KEY_PROG1);

	rc = input_register_device(input_device);
	if (rc)
		goto error_free_device;

	ts->wq = create_singlethread_workqueue("kworkqueue_ts");
	if (!ts->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}

	INIT_DELAYED_WORK(&ts->work, cy8ctma300_xy_worker);

	if (ts->pdata->power_on) {
		rc = ts->pdata->power_on(1);
		if (rc) {
			dev_err(&client->dev, "could not power-up\n");
			goto error_power_on;
		}
	}

	if (!ts->pdata->use_polling) {
		rc = request_irq(client->irq, cy8ctma300_irq,
					IRQF_TRIGGER_FALLING,
					client->dev.driver->name, ts);
		if (rc) {
			dev_err(&client->dev, "could not request irq\n");
			goto error_unreg_device;
		}
	} else {
		/* wait for 25sec before reading I2C due to
		 * hardware limitation
		 */
		queue_delayed_work(ts->wq, &ts->work, INITIAL_DELAY);
	}

	/* write to STATUS_REG to make sure hardware doesn't get
	 * locked up during boot
	 */
	rc = cy8ctma300_write_reg_u8(ts->client, STATUS_REG, UPDATE_COORDS);
	if (rc < 0) {
		dev_err(&ts->client->dev, "write failed, try once more\n");

		rc = cy8ctma300_write_reg_u8(ts->client, STATUS_REG,
				UPDATE_COORDS);
		if (rc < 0)
			dev_err(&ts->client->dev, "write failed, exiting\n");
	}
	return 0;
error_unreg_device:
	if (ts->pdata->power_on)
		rc = ts->pdata->power_on(0);
error_power_on:
	destroy_workqueue(ts->wq);
	input_unregister_device(input_device);
	input_device = NULL;
error_wq_create:
error_free_device:
	input_free_device(input_device);

	return rc;
}

#ifdef CONFIG_PM
static int cy8ctma300_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cy8ctma300 *ts = i2c_get_clientdata(client);
	int rc = 0;

	if (!ts->pdata->use_polling)
		disable_irq_nosync(ts->client->irq);

	rc = cancel_delayed_work_sync(&ts->work);

	if (rc && !ts->pdata->use_polling)
		enable_irq(ts->client->irq);

	if (ts->pdata->power_on) {
		rc = ts->pdata->power_on(0);
		if (rc) {
			dev_err(&client->dev, "unable to goto suspend\n");
			return rc;
		}
	}
	return 0;
}

static int cy8ctma300_resume(struct i2c_client *client)
{
	struct cy8ctma300 *ts = i2c_get_clientdata(client);
	int rc = 0;

	if (ts->pdata->power_on) {
		rc = ts->pdata->power_on(1);
		if (rc) {
			dev_err(&client->dev, "unable to resume\n");
			return rc;
		}
	}

	if (ts->pdata->use_polling)
		queue_delayed_work(ts->wq, &ts->work, TOUCHSCREEN_TIMEOUT);
	else
		enable_irq(ts->client->irq);

	return 0;
}
#else
#define cy8ctma300_resume NULL
#define cy8ctma300_suspend NULL
#endif

static int __devinit cy8ctma300_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct cy8ctma300 *ts;
	struct cy8ctma300_platform_data *pdata = client->dev.platform_data;
	int rc = 0;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	if (!pdata->nfingers || pdata->nfingers > 10) {
		dev_err(&client->dev, "Touches supported are 1 to 10\n");
		return -EINVAL;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	ts->pdata = pdata;
	i2c_set_clientdata(client, ts);

	ts->touch_data = kzalloc(ts->pdata->nfingers * BYTES_PER_TOUCH *
			sizeof(ts->touch_data) + TOUCH_META_INFO,
				 GFP_KERNEL);
	if (!ts->touch_data)
		goto error_touch_data_alloc;

	ts->prev_touches = 0;

	rc = cy8ctma300_init_ts(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "cy8ctma300 TS controller init failed\n");
		goto err_init_ts;
	}

	return 0;

err_init_ts:
	kfree(ts->touch_data);
error_touch_data_alloc:
	kfree(ts);
	i2c_set_clientdata(client, NULL);
	return rc;
}

static int __devexit cy8ctma300_remove(struct i2c_client *client)
{
	struct cy8ctma300 *ts = i2c_get_clientdata(client);
	int rc;

	rc = cancel_delayed_work_sync(&ts->work);

	if (rc && !ts->pdata->use_polling)
		enable_irq(ts->client->irq);

	if (!ts->pdata->use_polling)
		free_irq(client->irq, ts);

	destroy_workqueue(ts->wq);

	input_unregister_device(ts->input);

	if (ts->pdata->power_on)
		ts->pdata->power_on(0);

	kfree(ts->touch_data);
	kfree(ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id cy8ctma300_id[] = {
	{"cy8ctma300", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, cy8ctma300_id);

static struct i2c_driver cy8ctma300_driver = {
	.driver = {
		.name = "cy8ctma300",
		.owner = THIS_MODULE,
	},
	.probe		= cy8ctma300_probe,
	.remove		= __devexit_p(cy8ctma300_remove),
	.suspend	= cy8ctma300_suspend,
	.resume		= cy8ctma300_resume,
	.id_table	= cy8ctma300_id,
};

static int __init cy8ctma300_init(void)
{
	return i2c_add_driver(&cy8ctma300_driver);
}
/* Making this as late init to avoid power fluctuations
 * during LCD initialization.
 */
late_initcall(cy8ctma300_init);

static void __exit cy8ctma300_exit(void)
{
	return i2c_del_driver(&cy8ctma300_driver);
}
module_exit(cy8ctma300_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CY8CTMA300 touchscreen controller driver");
MODULE_AUTHOR("Cypress");
MODULE_ALIAS("platform:cy8ctma300");
