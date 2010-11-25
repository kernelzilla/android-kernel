/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
/*
 * ATLab FO1W-I2C Optical finger navigation driver.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/system.h>
#include <mach/gpio.h>
#include <mach/ofn_atlab.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.1");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Atlab Optical finger navigation driver");
MODULE_ALIAS("platform:ofn-atlab");

static void optnav_work_f(struct work_struct *work);

#define OPTNAV_SLAVE_ADDR_REG          0x0
#define OPTNAV_CONFIG_REG              0x1
#define OPTNAV_KEYS_STATUS_REG         0x2
#define OPTNAV_MOTION_X_REG            0x3
#define OPTNAV_MOTION_Y_REG            0x4
#define OPTNAV_FUNCTION_1_REG          0x6
#define OPTNAV_FUNCTION_2_REG          0x7
#define OPTNAV_MODE_REG                0x8

#define OPTNAV_CONFIG_DEFAULT          0x1F
#define OPTNAV_CONFIG_DEFAULT_MASK     0x7F
#define OPTNAV_FUNC1_DELAY_10MS        0x0
#define OPTNAV_FUNC1_TOUT_INVERTING    0x4
#define OPTNAV_FUNC1_NO_MOTION_ENABLE  0x2
#define OPTNAV_FUNC2_HOLDA             0x4
#define OPTNAV_FUNC2_HOLDB             0x1

#define QUALCOMM_VENDORID              0x5413

#define DRIVER_NAME                    "ofn_atlab"
#define DEVICE_NAME                    "fo1w"
#define OPTNAV_NAME                    "Atlab OFN"
#define OPTNAV_NAME_BUTTON_L           "OFN button l"
#define OPTNAV_NAME_BUTTON_R           "OFN button r"
#define OPTNAV_DEVICE                  "/dev/ofn"
#define OPTNAV_BUF_SIZE                2
#define OPTNAV_STABILIZE_DELAY_MS      30

struct optnav_data {
	struct input_dev  *optnav_dev;
	struct i2c_client *clientp;
	u32                irq_button_l;
	u32                irq_button_r;
	u32                gpio_button_l;
	u32                gpio_button_r;
	int              (*optnav_on) (void);
	void             (*optnav_off)(void);
	bool               rotate_xy;
	struct work_struct optnav_work;
};

static int optnav_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	int           rc;

	/* Device NAK's writes so they always appears to fail - can't  */
	/* check for errors on write. Read back value to confirm it    */
	/* was written correctly.                                      */
	i2c_smbus_write_byte_data(client, reg, data);
	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0) {
		dev_err(&client->dev,
			"optnav_i2c_write FAILED reading reg %d\n",
			reg);
		return rc;
	}

	if ((u8)rc != data) {
		dev_err(&client->dev,
			"optnav_i2c_write FAILED writing to reg %d\n",
			reg);
		rc = -EIO;
		return rc;
	}
	return 0;
}

static void optnav_work_f(struct work_struct *work)
{
	struct optnav_data     *dd =
		container_of(work, struct optnav_data, optnav_work);
	u8                      x_reg, y_reg;
	s8                      x_val, y_val;
	int                     rc, rc2;
	int                     delay = 0;

	if (dd->optnav_on != NULL) {
		rc = dd->optnav_on();
		if (rc)
			goto on_workf_exit;
		msleep(OPTNAV_STABILIZE_DELAY_MS);
	}

	/* Poll the device. While we keep getting new data, poll with */
	/* no additional delay. When we receive no new data,          */
	/* gradually reduce the time between polls. When we have gone */
	/* a significant time with no new data, turn off the device.  */
	do {
		if (delay)
			msleep(delay);
		/* The keys status reg must be read to unlatch the  */
		/* X, Y values, even though we don't need keys data */
		rc = i2c_smbus_read_byte_data(dd->clientp,
					      OPTNAV_KEYS_STATUS_REG);
		rc2 = i2c_smbus_read_word_data(dd->clientp,
					       OPTNAV_MOTION_X_REG);
		if ((rc < 0) || (rc2 < 0)) {
			dev_err(&dd->clientp->dev,
				"optnav_work_f: i2c read failed\n");
			goto on_workf_exit;
		}
		x_reg = rc2 & 0xFF;
		y_reg = rc2>>8 & 0xFF;

		if (!x_reg && !y_reg)
			delay += 10;
		else {
			delay = 0;
			if (dd->rotate_xy) {
				/* If the device is oriented incorrectly   */
				/* X and Y readings will be rotated - this */
				/* puts them back to natural orientation   */
				x_val = y_reg;
				y_val = -x_reg;
			} else {
				x_val = x_reg;
				y_val = y_reg;
			}
			input_report_rel(dd->optnav_dev, REL_X, x_val);
			input_report_rel(dd->optnav_dev, REL_Y, y_val);
			input_sync(dd->optnav_dev);
		}
		/* when delay reaches > 40ms we have had successive waits */
		/* of 10+20+30+40 = 100ms, as recommended in datasheet    */
	} while (delay <= 40);

on_workf_exit:
	if (dd->optnav_off != NULL)
		dd->optnav_off();
}

static int optnav_enable(struct optnav_data *dd)
{
	int rc = 0;

	if (dd->optnav_on != NULL)
		rc = dd->optnav_on();
	if (rc)
		goto on_enable_exit;

	rc = optnav_i2c_write(dd->clientp, OPTNAV_FUNCTION_1_REG,
		OPTNAV_FUNC1_DELAY_10MS | OPTNAV_FUNC1_TOUT_INVERTING |
		OPTNAV_FUNC1_NO_MOTION_ENABLE);

on_enable_exit:
	/* turn off device - it will generate interrupt when touched */
	if (dd->optnav_off != NULL)
		dd->optnav_off();
	return rc;
}

static irqreturn_t optnav_butl_interrupt(int irq, void *dev_id)
{
	struct optnav_data *dd = dev_id;
	u32 state;

	state = gpio_get_value(dd->gpio_button_l);
	input_report_key(dd->optnav_dev, BTN_LEFT, !state);
	input_sync(dd->optnav_dev);
	return IRQ_HANDLED;
}

static irqreturn_t optnav_butr_interrupt(int irq, void *dev_id)
{
	struct optnav_data *dd = dev_id;
	u32 state;

	state = gpio_get_value(dd->gpio_button_r);
	input_report_key(dd->optnav_dev, BTN_RIGHT, !state);
	input_sync(dd->optnav_dev);
	return IRQ_HANDLED;
}

static irqreturn_t optnav_interrupt(int irq, void *dev_id)
{
	struct optnav_data *dd = dev_id;

	schedule_work(&dd->optnav_work);
	return IRQ_HANDLED;
}

static int optnav_dev_open(struct input_dev *dev)
{
	int rc;
	struct optnav_data *dd = input_get_drvdata(dev);

	if (!dd->clientp) {
		printk(KERN_ERR "optnav_dev_open: no i2c adapter present\n");
		rc = -ENODEV;
		goto fail_on_open;
	}
	rc = optnav_enable(dd);
	if (rc)
		goto fail_on_open;

	rc = request_irq(dd->irq_button_l,
		       &optnav_butl_interrupt,
		       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		       OPTNAV_NAME_BUTTON_L,
		       dd);
	if (rc) {
		dev_err(&dd->clientp->dev,
		       "optnav_dev_open FAILED: request_irq rc=%d\n", rc);
		goto fail_on_open;
	}

	rc = request_irq(dd->irq_button_r,
		       &optnav_butr_interrupt,
		       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		       OPTNAV_NAME_BUTTON_R,
		       dd);
	if (rc) {
		dev_err(&dd->clientp->dev,
		       "optnav_dev_open FAILED: request_irq rc=%d\n", rc);
		goto fail_on_open1;
	}
	rc = request_irq(dd->clientp->irq,
		       &optnav_interrupt,
		       IRQF_TRIGGER_FALLING,
		       OPTNAV_NAME,
		       dd);
	if (rc) {
		dev_err(&dd->clientp->dev,
		       "optnav_dev_open FAILED: request_irq rc=%d\n", rc);
		goto fail_on_open2;
	}
	return 0;

fail_on_open2:
	free_irq(dd->irq_button_r, dd);
fail_on_open1:
	free_irq(dd->irq_button_l, dd);
fail_on_open:
	return rc;
}

static void optnav_dev_close(struct input_dev *dev)
{
	struct optnav_data *dd = input_get_drvdata(dev);

	free_irq(dd->irq_button_l, dd);
	free_irq(dd->irq_button_r, dd);
	free_irq(dd->clientp->irq, dd);
	cancel_work_sync(&dd->optnav_work);
}

static int __exit optnav_remove(struct i2c_client *client)
{
	struct optnav_data              *dd;
	struct ofn_atlab_platform_data  *pd;

	dd = i2c_get_clientdata(client);
	pd = client->dev.platform_data;
	input_unregister_device(dd->optnav_dev);
	if (pd->gpio_release != NULL)
		pd->gpio_release();
	kfree(dd);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int optnav_suspend(struct i2c_client *client,
			    pm_message_t state)
{
	struct optnav_data *dd;

	dd = i2c_get_clientdata(client);
	disable_irq(dd->irq_button_l);
	disable_irq(dd->irq_button_r);
	disable_irq(client->irq);
	cancel_work_sync(&dd->optnav_work);
	return 0;
}

static int optnav_resume(struct i2c_client *client)
{
	struct optnav_data *dd;

	dd = i2c_get_clientdata(client);
	enable_irq(dd->irq_button_l);
	enable_irq(dd->irq_button_r);
	enable_irq(client->irq);
	return 0;
}
#endif

static const struct i2c_device_id optnav_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, optnav_id);

static int optnav_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int                              rc;
	struct ofn_atlab_platform_data  *pd;
	struct optnav_data              *dd;

	dd = kzalloc(sizeof *dd, GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	i2c_set_clientdata(client, dd);

	rc = i2c_smbus_read_byte_data(client, OPTNAV_CONFIG_REG);
	if (rc < 0)
		goto probe_free_exit;

	if ((rc & OPTNAV_CONFIG_DEFAULT_MASK) != OPTNAV_CONFIG_DEFAULT) {
		rc = -ENODEV;
		goto probe_free_exit;
	}

	dd->clientp = client;
	pd = client->dev.platform_data;
	if (!pd) {
		dev_err(&client->dev,
			"optnav_probe: platform data not set\n");
		rc = -EFAULT;
		goto probe_free_exit;
	}

	dd->irq_button_l = pd->irq_button_l;
	dd->irq_button_r = pd->irq_button_r;
	dd->gpio_button_l = pd->gpio_button_l;
	dd->gpio_button_r = pd->gpio_button_r;
	dd->optnav_on = pd->optnav_on;
	dd->optnav_off = pd->optnav_off;
	dd->rotate_xy = pd->rotate_xy;

	if (pd->gpio_setup == NULL)
		goto probe_free_exit;
	rc = pd->gpio_setup();
	if (rc)
		goto probe_free_exit;

	dd->optnav_dev = input_allocate_device();
	if (!dd->optnav_dev) {
		rc = -ENOMEM;
		goto probe_fail_in_alloc;
	}

	input_set_drvdata(dd->optnav_dev, dd);
	dd->optnav_dev->open       = optnav_dev_open;
	dd->optnav_dev->close      = optnav_dev_close;
	dd->optnav_dev->name       = OPTNAV_NAME;
	dd->optnav_dev->phys       = OPTNAV_DEVICE;
	dd->optnav_dev->id.bustype = BUS_I2C;
	dd->optnav_dev->id.vendor  = QUALCOMM_VENDORID;
	dd->optnav_dev->id.product = 1;
	dd->optnav_dev->id.version = 1;
	__set_bit(EV_REL,    dd->optnav_dev->evbit);
	__set_bit(REL_X,     dd->optnav_dev->relbit);
	__set_bit(REL_Y,     dd->optnav_dev->relbit);
	__set_bit(EV_KEY,    dd->optnav_dev->evbit);
	__set_bit(BTN_LEFT,  dd->optnav_dev->keybit);
	__set_bit(BTN_RIGHT, dd->optnav_dev->keybit);

	rc = input_register_device(dd->optnav_dev);
	if (rc) {
		dev_err(&client->dev,
			"optnav_probe: input_register_device rc=%d\n",
		       rc);
		goto probe_fail_reg_dev;
	}
	INIT_WORK(&dd->optnav_work, optnav_work_f);
	return 0;

probe_fail_reg_dev:
	input_free_device(dd->optnav_dev);
probe_fail_in_alloc:
	if (pd->gpio_release != NULL)
		pd->gpio_release();
probe_free_exit:
	kfree(dd);
	i2c_set_clientdata(client, NULL);
probe_exit:
	return rc;
}

static struct i2c_driver optnav_driver = {
	.driver = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
	.probe   = optnav_probe,
	.remove  =  __exit_p(optnav_remove),
#ifdef CONFIG_PM
	.suspend = optnav_suspend,
	.resume  = optnav_resume,
#endif
	.id_table = optnav_id,
};

static int __init optnav_init(void)
{
	int rc;

	rc = i2c_add_driver(&optnav_driver);
	if (rc)
		printk(KERN_ERR "optnav_init FAILED: i2c_add_driver rc=%d\n",
		       rc);
	return rc;
}

static void __exit optnav_exit(void)
{
	i2c_del_driver(&optnav_driver);
}

module_init(optnav_init);
module_exit(optnav_exit);
