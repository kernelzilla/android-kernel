/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/system.h>
#include <mach/gpio.h>
#include <mach/msm_touchpad.h>

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_DESCRIPTION("MSM Touchpad driver");

static int         touchpad_probe(struct i2c_client *client,
				  const struct i2c_device_id *id);
static void        touchpad_work_f(struct work_struct *work);
static irqreturn_t touchpad_interrupt(int irq, void *dev_id);

#define TOUCHPAD_RAP_READ              0xA0
#define TOUCHPAD_RAP_WRITE             0x80

#define TOUCHPAD_REG_STATUS            0x02
#define TOUCHPAD_REG_CALIBRATE         0x03
#define TOUCHPAD_REG_COMMAND           0x04
#define TOUCHPAD_REG_Z_IDLE            0x07
#define TOUCHPAD_REG_ABS_X             0x08
#define TOUCHPAD_REG_ABS_Z             0x0B

#define TOUCHPAD_STATUS_SDR            0x04
#define TOUCHPAD_CMD_ABSOLUTE_ENABLE   0x01
#define TOUCHPAD_Z_TOUCH               0x80
#define TOUCHPAD_CALIBRATE_START       0x02

#define TOUCHPAD_MAX_TAP_DIAMETER      100
#define TOUCHPAD_TOUCH_COUNT           16
#define TOUCHPAD_NO_TOUCH_COUNT        38
#define TOUCHPAD_Z_IDLE_COUNT          (TOUCHPAD_NO_TOUCH_COUNT + 5)

#define TOUCHPAD_CALIBRATION_TIMEOUT   (msecs_to_jiffies(200))
#define TOUCHPAD_BUF_SIZE              4

#define QUALCOMM_VENDORID              0x5413

#define TOUCHPAD_NAME                  "MSM Touchpad"
#define TOUCHPAD_DEVICE                "/dev/tpad"

struct touchpad_data {
	struct input_dev *touchpad_dev;
	bool              calibrating;
	bool              calibrated;
	u32               x_min, y_min;
	u32               x_max, y_max;
	u32               x_last, y_last;
	u32               touchpad_gpio;
	u32               touchpad_suspend_gpio;
	struct i2c_client *clientp;
};

static struct touchpad_data *tp_data;
static	DECLARE_COMPLETION(calibration_complete);
static	DECLARE_WORK(touchpad_work, touchpad_work_f);

#ifdef CONFIG_PM
static int touchpad_suspend(struct i2c_client *client,
			    pm_message_t state)
{
	struct touchpad_data   *dd = tp_data;
	gpio_direction_output(dd->touchpad_suspend_gpio, 1);
	return 0;
}

static int touchpad_resume(struct i2c_client *client)
{
	struct touchpad_data   *dd = tp_data;
	gpio_direction_output(dd->touchpad_suspend_gpio, 0);
	return 0;
}
#endif

static int touchpad_i2c_read(struct i2c_client *client,
			     int reg, u8 *buf, int count)
{
	int rc;
	int ret = 0;

	buf[0] = TOUCHPAD_RAP_READ | reg;
	rc = i2c_master_send(client, buf, 1);
	if (rc != 1) {
		dev_err(&client->dev,
		       "touchpad_i2c_read FAILED: read of register %d\n", reg);
		ret = -1;
		goto tp_i2c_rd_exit;
	}
	rc = i2c_master_recv(client, buf, count);
	if (rc != count) {
		dev_err(&client->dev,
		       "touchpad_i2c_read FAILED: read %d bytes from reg %d\n",
		       count, reg);
		ret = -1;
	}

tp_i2c_rd_exit:
	return ret;
}

static int touchpad_i2c_write(struct i2c_client *client, int reg, u8 data)
{
	u8            buf[2];
	int           rc;
	int           ret = 0;

	buf[0] = TOUCHPAD_RAP_WRITE | reg;
	buf[1] = data;
	rc = i2c_master_send(client, buf, 2);
	if (rc != 2) {
		dev_err(&client->dev,
		       "touchpad_i2c_write FAILED: writing to reg %d\n", reg);
		ret = -1;
	}
	return ret;
}

static int touchpad_enable(struct touchpad_data *dd)
{
	u8            buf[1];
	u8            data;
	int           rc;
	int           comp_status;

	if (!dd->calibrated) {
		dd->calibrating = 1;
		rc = touchpad_i2c_write(dd->clientp, TOUCHPAD_REG_CALIBRATE,
					TOUCHPAD_CALIBRATE_START);
		if (rc)
			goto tp_en_err_exit;
		comp_status = wait_for_completion_timeout(
			&calibration_complete, TOUCHPAD_CALIBRATION_TIMEOUT);
		if (!comp_status)
			dev_info(&dd->clientp->dev,
				 "touchpad_enable: calibration timeout\n");
		dd->calibrating = 0;
		dd->calibrated = 1;
	}
	/* Read command register */
	rc = touchpad_i2c_read(dd->clientp, TOUCHPAD_REG_COMMAND, buf, 1);
	if (rc)
		goto tp_en_err_exit;
	data = buf[0];

	/* OR in init bits & enable touchpad */
	rc = touchpad_i2c_write(dd->clientp, TOUCHPAD_REG_COMMAND,
				data | TOUCHPAD_CMD_ABSOLUTE_ENABLE);
	if (rc)
		goto tp_en_err_exit;
	/* Clear data ready */
	rc = touchpad_i2c_read(dd->clientp, TOUCHPAD_REG_ABS_Z, buf, 1);
	if (rc)
		goto tp_en_err_exit;

	/* read back cmd reg to make sure write worked */
	rc = touchpad_i2c_read(dd->clientp, TOUCHPAD_REG_COMMAND, buf, 1);
	if (rc)
		goto tp_en_err_exit;
	if (buf[0] != (data | TOUCHPAD_CMD_ABSOLUTE_ENABLE)) {
		dev_err(&dd->clientp->dev,
		       "touchpad_enable FAILED: enable read 0x%x "
		       "expected 0x%x\n",
		       buf[0], data | TOUCHPAD_CMD_ABSOLUTE_ENABLE);
		goto tp_en_err_exit;
	}

	rc = touchpad_i2c_write(dd->clientp, TOUCHPAD_REG_Z_IDLE,
				TOUCHPAD_Z_IDLE_COUNT);
	if (rc)
		goto tp_en_err_exit;

	return 0;
tp_en_err_exit:
	return -1;
}

/* Touchpad work function. Reads data from touchpad via i2c. */
static void touchpad_work_f(struct work_struct *work)
{
	struct touchpad_data   *dd = tp_data;
	static bool             being_touched;
	static bool             button_down;
	static unsigned int     num_non_touch_ints;
	static unsigned int     num_touch_ints;
	static s32              x_save[TOUCHPAD_TOUCH_COUNT+1];
	static s32              y_save[TOUCHPAD_TOUCH_COUNT+1];
	static unsigned int     save_index;
	u8                      buf[TOUCHPAD_BUF_SIZE];
	u32                     x_val, y_val;
	int                     rc;
	int                     touch;
	int                     report_xy;
	int                     i;
	s32                     x_rel, y_rel;

	if (dd->calibrating) {
		dd->calibrating = 0;
		complete(&calibration_complete);
		goto tp_wk_exit;
	}

	rc = touchpad_i2c_read(dd->clientp, TOUCHPAD_REG_ABS_X, buf,
			       TOUCHPAD_BUF_SIZE);
	if (rc)
		goto tp_wk_exit;
	x_val = (int)(buf[0] | ((buf[2]&0xF0)<<4));
	y_val = (int)(buf[1] | ((buf[2]&0x0F)<<8));
	touch = (buf[3]&TOUCHPAD_Z_TOUCH)>>7;

	report_xy = 1;
	if (!being_touched) {
		if (!touch) {
			num_non_touch_ints++;
			/* no-touch timeout after tap */
			if ((num_non_touch_ints == TOUCHPAD_NO_TOUCH_COUNT) &&
			    button_down){
				input_report_key(dd->touchpad_dev,
						 BTN_MOUSE, 0);
				input_sync(dd->touchpad_dev);
				button_down = 0;
			}
			goto tp_wk_exit;
		}
		/* transition from no-touch to touch */
		being_touched = 1;
		num_touch_ints = 1;
		dd->x_min = 0x7fffffff;
		dd->x_max = 0;
		dd->y_min = 0x7fffffff;
		dd->y_max = 0;
		x_rel = 0;
		y_rel = 0;
		save_index = 0;
	} else if (touch) {
		/* middle of a touch */
		num_touch_ints++;
		dd->x_min = min(dd->x_min, x_val);
		dd->x_max = max(dd->x_max, x_val);
		dd->y_min = min(dd->y_min, y_val);
		dd->y_max = max(dd->y_max, y_val);
		x_rel = x_val - dd->x_last;
		y_rel = y_val - dd->y_last;
		/* during a tap, save non-0 movements for drift compensation */
		if (((x_rel != 0) || (y_rel != 0)) &&
		    (num_touch_ints <= TOUCHPAD_TOUCH_COUNT)) {
			x_save[save_index] = x_rel;
			y_save[save_index++] = y_rel;
		}
	} else {
		/* transition from touch to no-touch */
		num_non_touch_ints = 1;
		report_xy = 0;
		being_touched = 0;
		if ((num_touch_ints <= TOUCHPAD_TOUCH_COUNT) &&
		    ((dd->x_max - dd->x_min) <= TOUCHPAD_MAX_TAP_DIAMETER) &&
		    ((dd->y_max - dd->y_min) <= TOUCHPAD_MAX_TAP_DIAMETER)) {
			/* compensate for drift during tap by */
			/* restoring to original position     */
			for (i = 0; i < save_index; i++) {
				if (x_save[i])
					input_report_rel(dd->touchpad_dev,
							 REL_X, -x_save[i]);
				if (y_save[i])
					input_report_rel(dd->touchpad_dev,
							 REL_Y, -y_save[i]);
				input_sync(dd->touchpad_dev);
			}
			if (button_down) {
				/* Double click: button is still down from   */
				/* first click - put it up before signalling */
				/* down for 2nd click                        */
				input_report_key(dd->touchpad_dev,
						 BTN_MOUSE, 0);
				input_sync(dd->touchpad_dev);
			}
			input_report_key(dd->touchpad_dev, BTN_MOUSE, 1);
			button_down = 1;
			input_sync(dd->touchpad_dev);
		}
	}
	if (report_xy) {
		if (x_rel)
			input_report_rel(dd->touchpad_dev, REL_X, x_rel);
		if (y_rel)
			input_report_rel(dd->touchpad_dev, REL_Y, y_rel);
		input_sync(dd->touchpad_dev);
	}
	if (touch) {
		dd->x_last = x_val;
		dd->y_last = y_val;
	}

tp_wk_exit:
	return;
}

static int touchpad_dev_open(struct input_dev *dev)
{
	int rc;
	struct touchpad_data *dd = input_get_drvdata(dev);

	if (!dd->clientp) {
		printk(KERN_ERR "touchpad_dev_open: no i2c adapter present\n");
		rc = -ENODEV;
		goto fail_irq;
	}
	rc = gpio_direction_input(dd->touchpad_gpio);
	if (rc) {
		dev_err(&dd->clientp->dev,
		       "touchpad_dev_open FAILED: gpio_direction_input(%d) "
		       "rc=%d\n", dd->touchpad_gpio, rc);
		goto fail_irq;
	}
	rc = request_irq(dd->clientp->irq,
		       &touchpad_interrupt,
		       IRQF_TRIGGER_RISING,
		       "Touchpad",
		       0);
	if (rc) {
		dev_err(&dd->clientp->dev,
		       "touchpad_dev_open FAILED: request_irq rc=%d\n", rc);
		goto fail_irq;
	}
	rc = touchpad_enable(dd);
	if (rc) {
		dev_err(&dd->clientp->dev,
		       "touchpad_dev_open FAILED: touchpad_enable rc=%d\n", rc);
		goto fail_tp_enable;
	}
	return 0;

fail_tp_enable:
	free_irq(dd->clientp->irq, NULL);
fail_irq:
	return rc;
}

static void touchpad_dev_close(struct input_dev *dev)
{
	struct touchpad_data *dd = input_get_drvdata(dev);
	free_irq(dd->clientp->irq, NULL);
}

static irqreturn_t touchpad_interrupt(int irq, void *dev_id)
{
	schedule_work(&touchpad_work);
	return IRQ_HANDLED;
}

static int __exit touchpad_remove(struct i2c_client *client)
{
	struct touchpad_data              *dd;
	struct msm_touchpad_platform_data *pd;

	dd = i2c_get_clientdata(client);
	pd = client->dev.platform_data;
	input_unregister_device(dd->touchpad_dev);
	if (pd->gpio_shutdown != NULL)
		pd->gpio_shutdown();
	kfree(dd);
	i2c_set_clientdata(client, NULL);
	tp_data = NULL;

	return 0;
}

static const struct i2c_device_id touchpad_id[] = {
	{ "glidesensor", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, touchpad_id);

static struct i2c_driver touchpad_driver = {
	.driver = {
		.name   = "msm_touchpad",
		.owner  = THIS_MODULE,
	},
	.probe    = touchpad_probe,
	.remove   =  __exit_p(touchpad_remove),
#ifdef CONFIG_PM
	.suspend  = touchpad_suspend,
	.resume   = touchpad_resume,
#endif
	.id_table = touchpad_id,
};

static int touchpad_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	u8                                 buf[2];
	int                                count;
	int                                rc;
	struct msm_touchpad_platform_data *pd;
	struct touchpad_data              *dd;

	if (tp_data) {
		dev_err(&client->dev,
			"touchpad_probe: only a single touchpad supported\n");
		rc = -EPERM;
		goto probe_exit;
	}

	dd = kzalloc(sizeof *dd, GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}
	tp_data = dd;
	i2c_set_clientdata(client, dd);

	buf[0] = TOUCHPAD_RAP_READ | TOUCHPAD_REG_COMMAND;
	count = 1;
	rc = i2c_master_send(client, buf, count);
	if (count != rc)
		goto probe_free_exit;
	dd->clientp = client;
	client->driver = &touchpad_driver;
	pd = client->dev.platform_data;
	if (!pd) {
		dev_err(&client->dev,
			"touchpad_probe: platform data not set\n");
		rc = -EFAULT;
		goto probe_free_exit;
	}

	dd->touchpad_gpio = pd->gpioirq;
	dd->touchpad_suspend_gpio = pd->gpiosuspend;

	if (pd->gpio_setup == NULL)
		goto probe_free_exit;
	rc = pd->gpio_setup();
	if (rc)
		goto probe_free_exit;

	dd->touchpad_dev = input_allocate_device();
	if (!dd->touchpad_dev) {
		rc = -ENOMEM;
		goto probe_gpiosus_exit;
	}
	input_set_drvdata(dd->touchpad_dev, dd);
	dd->touchpad_dev->open       = touchpad_dev_open;
	dd->touchpad_dev->close      = touchpad_dev_close;
	dd->touchpad_dev->name       = TOUCHPAD_NAME;
	dd->touchpad_dev->phys       = TOUCHPAD_DEVICE;
	dd->touchpad_dev->id.bustype = BUS_I2C;
	dd->touchpad_dev->id.vendor  = QUALCOMM_VENDORID;
	dd->touchpad_dev->id.product = 1;
	dd->touchpad_dev->id.version = 2;
	__set_bit(EV_REL,    dd->touchpad_dev->evbit);
	__set_bit(REL_X,     dd->touchpad_dev->relbit);
	__set_bit(REL_Y,     dd->touchpad_dev->relbit);
	__set_bit(EV_KEY,    dd->touchpad_dev->evbit);
	__set_bit(BTN_MOUSE, dd->touchpad_dev->keybit);

	rc = input_register_device(dd->touchpad_dev);
	if (rc) {
		dev_err(&client->dev,
			"touchpad_probe: input_register_device rc=%d\n",
		       rc);
		goto probe_fail_reg_dev;
	}

	return 0;

probe_fail_reg_dev:
	input_free_device(dd->touchpad_dev);
probe_gpiosus_exit:
	if (pd->gpio_shutdown != NULL)
		pd->gpio_shutdown();
probe_free_exit:
	kfree(dd);
	tp_data = NULL;
	i2c_set_clientdata(client, NULL);
probe_exit:
	return rc;
}

static int __init touchpad_init(void)
{
	int rc;

	rc = i2c_add_driver(&touchpad_driver);
	if (rc) {
		printk(KERN_ERR "touchpad_init FAILED: i2c_add_driver rc=%d\n",
		       rc);
		goto init_exit;
	}

	return 0;

init_exit:
	return rc;
}

static void __exit touchpad_exit(void)
{
	i2c_del_driver(&touchpad_driver);
}

module_init(touchpad_init);
module_exit(touchpad_exit);
