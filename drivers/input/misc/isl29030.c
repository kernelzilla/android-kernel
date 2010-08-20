/*
 * Copyright (C) 2009 Motorola, Inc.
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

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/isl29030.h>
#include <linux/types.h>



struct isl29030_data {
	struct input_dev *idev;
	struct input_dev *adev;
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct work_struct wq;
	struct workqueue_struct *working_queue;
	struct isl29030_platform_data *als_ir_pdata;
	struct early_suspend		early_suspend;
	struct isl29030_zone_conv isl29030_low_zone_info[255];
	struct isl29030_zone_conv isl29030_high_zone_info[255];
	uint8_t zone;
	uint8_t low_lux_level;
	uint8_t prox_detect;
	atomic_t enabled;
	int irq;
};

struct isl29030_data *isl29030_misc_data;

struct isl29030_reg {
	const char *name;
	uint8_t reg;
} isl29030_regs[] = {
	{ "CHIP_ID",			ISL29030_CHIPID },
	{ "CONFIGURE",          ISL29030_CONFIGURE },
	{ "INTERRUPT",          ISL29030_INTERRUPT },
	{ "PROX_LT",			ISL29030_PROX_LT },
	{ "PROX_HT",			ISL29030_PROX_HT },
	{ "ALS_IR_TH1",			ISL29030_ALSIR_TH1 },
	{ "ALS_IR_TH2",			ISL29030_ALSIR_TH2 },
	{ "ALS_IR_TH3",         ISL29030_ALSIR_TH3 },
	{ "PROX_DATA",          ISL29030_PROX_DATA },
	{ "ALS_IR_DT1",         ISL29030_ALSIR_DT1 },
	{ "ALS_IR_DT2",         ISL29030_ALSIR_DT2 },
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29030_early_suspend(struct early_suspend *handler);
static void isl29030_late_resume(struct early_suspend *handler);
#endif

static uint32_t isl29030_debug = 0x03;
module_param_named(als_debug, isl29030_debug, uint, 0664);

static int isl29030_read_reg(struct isl29030_data *als_ir_data, uint8_t reg,
		   uint8_t *value)
{
	int error = 0;
	int i = 0;
	uint8_t dest_buffer;

	if (!value) {
		pr_err("%s: invalid value pointer\n", __func__);
		return -EINVAL;
	}
	do {
		dest_buffer = reg;
		error = i2c_master_send(als_ir_data->client, &dest_buffer, 1);
		if (error == 1) {
			error = i2c_master_recv(als_ir_data->client,
				&dest_buffer, LD_ISL29030_ALLOWED_R_BYTES);
		}
		if (error != LD_ISL29030_ALLOWED_R_BYTES) {
			pr_err("%s: read[%i] failed: %d\n", __func__, i, error);
			msleep_interruptible(LD_ISL29030_I2C_RETRY_DELAY);
		}
	} while ((error != LD_ISL29030_ALLOWED_R_BYTES) &&
			((++i) < LD_ISL29030_MAX_RW_RETRIES));

	if (error == LD_ISL29030_ALLOWED_R_BYTES) {
		error = 0;
		*value = dest_buffer;
	}

	return error;
}

static int isl29030_write_reg(struct isl29030_data *als_ir_data,
							uint8_t reg,
							uint8_t value)
{
	uint8_t buf[LD_ISL29030_ALLOWED_W_BYTES] = { reg, value };
	int bytes;
	int i = 0;

	do {
		bytes = i2c_master_send(als_ir_data->client, buf,
					LD_ISL29030_ALLOWED_W_BYTES);

		if (bytes != LD_ISL29030_ALLOWED_W_BYTES) {
			pr_err("%s: write %d failed: %d\n", __func__, i, bytes);
			msleep_interruptible(LD_ISL29030_I2C_RETRY_DELAY);
		}
	} while ((bytes != (LD_ISL29030_ALLOWED_W_BYTES))
		 && ((++i) < LD_ISL29030_MAX_RW_RETRIES));

	if (bytes != LD_ISL29030_ALLOWED_W_BYTES) {
		pr_err("%s: i2c_master_send error\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int ld_isl29030_init_registers(struct isl29030_data *data)
{
	if (isl29030_write_reg(data, ISL29030_CONFIGURE,
			data->als_ir_pdata->configure) ||
		isl29030_write_reg(data, ISL29030_INTERRUPT,
			data->als_ir_pdata->interrupt_cntrl) ||
		isl29030_write_reg(data, ISL29030_PROX_LT,
			data->als_ir_pdata->prox_lower_threshold) ||
		isl29030_write_reg(data, ISL29030_PROX_HT,
			data->als_ir_pdata->prox_higher_threshold) ||
		isl29030_write_reg(data, ISL29030_ALSIR_TH1,
			data->als_ir_pdata->als_ir_low_threshold) ||
		isl29030_write_reg(data, ISL29030_ALSIR_TH2,
			data->als_ir_pdata->als_ir_high_low_threshold) ||
		isl29030_write_reg(data, ISL29030_ALSIR_TH3,
			data->als_ir_pdata->als_ir_high_threshold)) {
		pr_err("%s:Register initialization failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

irqreturn_t ld_isl29030_irq_handler(int irq, void *dev)
{
	struct isl29030_data *als_ir_data = dev;

	disable_irq(als_ir_data->client->irq);
	queue_work(als_ir_data->working_queue, &als_ir_data->wq);

	return IRQ_HANDLED;
}

static int isl29030_read_adj_als(struct isl29030_data *isl)
{
	int ret;
	int i;
	int lux = 0;
	u8 als_lower;
	u8 als_upper;
	u16 als_read_data = 0;
	u8 configure_reg;

	ret = isl29030_read_reg(isl, ISL29030_ALSIR_DT1, &als_lower);
	if (ret != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
	       __func__, ret);
		return -1;
	}
	ret = isl29030_read_reg(isl, ISL29030_ALSIR_DT2, &als_upper);
	if (ret != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
	       __func__, ret);
		return -1;
	}

	als_read_data = (als_upper << 8);
	als_read_data |= als_lower;
	if (isl29030_debug & 1)
		pr_info("%s:Data read from ALS 0x%X\n",
		__func__, als_read_data);

	if ((als_read_data < ISL29030_LOW_LUX_TRIGGER) &&
		(!isl->low_lux_level)) {
			isl->low_lux_level = 1;

			ret = isl29030_read_reg(isl, ISL29030_CONFIGURE,
				&configure_reg);
			if (ret != 0) {
				pr_err("%s:%s: %d\n", __func__,
					"Unable to read interrupt register",
					ret);
			}
			ret = isl29030_write_reg(isl, ISL29030_CONFIGURE,
			     (configure_reg | 0x02));

	}
	/* Adjust the ALS threshold to a new window */
	if (isl->low_lux_level) {
		for (i = 0; i < isl->als_ir_pdata->num_of_low_zones; i++) {
			struct isl29030_zone_conv info =
				isl->isl29030_low_zone_info[i];
			struct isl29030_als_zone_data als_low_lux =
				isl->als_ir_pdata->als_low_lux[i];

			if (als_read_data > info.upper_threshold) {
				continue;
			} else if (als_read_data < info.lower_threshold) {
				continue;
			} else if ((als_read_data <= info.upper_threshold) &&
				(als_read_data >= info.lower_threshold)) {
				int num = isl->als_ir_pdata->num_of_low_zones;

				if (isl29030_debug & 1)
					pr_info("%s:%s %i\n", __func__,
						"Setting next window to", i);

				if (i == num - 1) {
					isl->low_lux_level = 0;
					ret = isl29030_read_reg(isl,
						ISL29030_CONFIGURE,
						&configure_reg);
					if (ret != 0) {
						pr_err("%s:%s%s:%d\n",
							__func__,
							"Unable to read ",
							"interrupt register",
							ret);
					}
					ret = isl29030_write_reg(isl,
						ISL29030_CONFIGURE,
						(configure_reg & 0xfd));
				}
				isl29030_write_reg(isl, ISL29030_ALSIR_TH1,
					als_low_lux.als_low_threshold);
				isl29030_write_reg(isl, ISL29030_ALSIR_TH2,
					als_low_lux.als_high_low_threshold);
				isl29030_write_reg(isl, ISL29030_ALSIR_TH3,
					als_low_lux.als_high_threshold);
			}
		}
		if (isl->als_ir_pdata->lens_percent_t)
			lux = ((625 / isl->als_ir_pdata->lens_percent_t) *
				(als_read_data)) / 100;

	} else {
		for (i = 0; i < isl->als_ir_pdata->num_of_high_zones; i++) {
			struct isl29030_zone_conv info =
				isl->isl29030_high_zone_info[i];
			struct isl29030_als_zone_data als_high_lux =
				isl->als_ir_pdata->als_high_lux[i];

			if (als_read_data > info.upper_threshold) {
				continue;
			} else if (als_read_data < info.lower_threshold) {
				continue;
			} else if ((als_read_data <= info.upper_threshold) &&
				(als_read_data >= info.lower_threshold)) {
				if (isl29030_debug & 1)
					pr_info("%s:%s %i\n", __func__,
						"Setting next window to ", i);
				isl29030_write_reg(isl, ISL29030_ALSIR_TH1,
					als_high_lux.als_low_threshold);
				isl29030_write_reg(isl, ISL29030_ALSIR_TH2,
					als_high_lux.als_high_low_threshold);
				isl29030_write_reg(isl, ISL29030_ALSIR_TH3,
					als_high_lux.als_high_threshold);
			}
		}
		if (isl->als_ir_pdata->lens_percent_t)
			lux = ((10000 / isl->als_ir_pdata->lens_percent_t) *
				(als_read_data)) / 100;
	}

	if (isl29030_debug & 1)
		pr_info("%s:Reporting LUX %d\n",
					__func__, lux);
	return lux;

}
static void isl29030_report_input(struct isl29030_data *isl)
{
	int ret = 0;
	int lux_val;
	u8 als_prox_int;
	u8 prox_data;

	ret = isl29030_read_reg(isl, ISL29030_INTERRUPT, &als_prox_int);
	if (ret != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
		       __func__, ret);
		return;
	}
	if ((als_prox_int & ISL29030_PROX_MASK) ||
		(isl->prox_detect)) {
		/* Report proximity here */
		ret = isl29030_read_reg(isl, ISL29030_PROX_DATA, &prox_data);
		if (ret != 0) {
			pr_err("%s:Unable to read interrupt register: %d\n",
		       __func__, ret);
		}
		if (isl29030_debug & 2)
			pr_err("%s:Data returned for PROX 0x%X\n",
			__func__, prox_data);

		if (prox_data > isl->als_ir_pdata->prox_higher_threshold) {
			input_report_abs(isl->idev, ABS_DISTANCE,
				PROXIMITY_NEAR);
			isl->prox_detect = 1;
			if (isl29030_debug & 2)
				pr_err("%s:Prox near\n", __func__);
		} else {
			input_report_abs(isl->idev, ABS_DISTANCE,
				PROXIMITY_FAR);
			isl->prox_detect = 0;
			if (isl29030_debug & 2)
				pr_err("%s:Prox far\n", __func__);
		}
		input_sync(isl->idev);
	}

	if (als_prox_int & ISL29030_ALS_MASK) {
		lux_val = isl29030_read_adj_als(isl);
		if (lux_val >= 0) {
			input_event(isl->adev, EV_LED, LED_MISC, lux_val);
			input_sync(isl->adev);
		}
	}
	isl29030_write_reg(isl, ISL29030_INTERRUPT,
	     (als_prox_int & 0xf7));

}

static void isl29030_device_power_off(struct isl29030_data *isl)
{
	int err;
	u8 configure_reg = isl->als_ir_pdata->configure & 0x7f;

	err = isl29030_write_reg(isl, ISL29030_CONFIGURE,
			     configure_reg);
	if (err)
		pr_err("%s:Unable to turn off prox: %d\n",
		       __func__, err);

}

static int isl29030_device_power_on(struct isl29030_data *isl)
{
	int err;
	u8 configure_reg = isl->als_ir_pdata->configure | 0x84;

	err = isl29030_write_reg(isl, ISL29030_CONFIGURE,
			     configure_reg);
	if (err)
		pr_err("%s:Unable to turn on prox: %d\n",
		       __func__, err);

	return err;
}

int isl29030_enable(struct isl29030_data *isl)
{
	int err;

	if (!atomic_cmpxchg(&isl->enabled, 0, 1)) {
		err = isl29030_device_power_on(isl);
		if (err) {
			atomic_set(&isl->enabled, 0);
			return err;
		}
	}
	return 0;
}

int isl29030_disable(struct isl29030_data *isl)
{
	if (atomic_cmpxchg(&isl->enabled, 1, 0))
		isl29030_device_power_off(isl);

	return 0;
}

static int isl29030_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = isl29030_misc_data;

	return 0;
}

static int isl29030_misc_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;
	struct isl29030_data *isl = file->private_data;

	switch (cmd) {
	case ISL29030_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			isl29030_enable(isl);
		else
			isl29030_disable(isl);

		break;

	case ISL29030_IOCTL_GET_ENABLE:
		enable = atomic_read(&isl->enabled);
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations isl29030_misc_fops = {
	.owner = THIS_MODULE,
	.open = isl29030_misc_open,
	.ioctl = isl29030_misc_ioctl,
};

static struct miscdevice isl29030_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FOPS_ISL29030_NAME,
	.fops = &isl29030_misc_fops,
};

static ssize_t ld_isl29030_registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client,
						 dev);
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);
	unsigned i, n, reg_count;
	uint8_t value;

	reg_count = sizeof(isl29030_regs) / sizeof(isl29030_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		isl29030_read_reg(als_ir_data, isl29030_regs[i].reg, &value);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       isl29030_regs[i].name,
			       value);
	}

	return n;
}

static ssize_t ld_isl29030_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client,
						 dev);
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);
	unsigned i, reg_count, value;
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%30s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(isl29030_regs) / sizeof(isl29030_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, isl29030_regs[i].name)) {
			error = isl29030_write_reg(als_ir_data,
				isl29030_regs[i].reg,
				value);
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return -1;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, 0644, ld_isl29030_registers_show,
		ld_isl29030_registers_store);

void ld_isl29030_work_queue(struct work_struct *work)
{
	struct isl29030_data *als_ir_data =
	    container_of(work, struct isl29030_data, wq);

	isl29030_report_input(als_ir_data);
	enable_irq(als_ir_data->client->irq);
}

static void isl29030_convert_zones(struct isl29030_data *isl)
{
	int i = 0;

	for (i = 0; i < isl->als_ir_pdata->num_of_low_zones; i++) {
		struct isl29030_als_zone_data als_low_lux =
			isl->als_ir_pdata->als_low_lux[i];

		isl->isl29030_low_zone_info[i].lower_threshold =
			als_low_lux.als_low_threshold;
		isl->isl29030_low_zone_info[i].lower_threshold |=
			(als_low_lux.als_high_low_threshold & 0x0f) << 8;
		isl->isl29030_low_zone_info[i].upper_threshold =
			(als_low_lux.als_high_low_threshold & 0xf0) >> 4;
		isl->isl29030_low_zone_info[i].upper_threshold |=
			als_low_lux.als_high_threshold << 4;
		pr_err("%s:Element %i Upper 0x%X Lower 0x%X\n", __func__, i,
			isl->isl29030_low_zone_info[i].upper_threshold,
			isl->isl29030_low_zone_info[i].lower_threshold);
	}
	for (i = 0; i < isl->als_ir_pdata->num_of_high_zones; i++) {
		struct isl29030_als_zone_data als_high_lux =
			isl->als_ir_pdata->als_high_lux[i];

		isl->isl29030_high_zone_info[i].lower_threshold =
			als_high_lux.als_low_threshold;
		isl->isl29030_high_zone_info[i].lower_threshold |=
			(als_high_lux.als_high_low_threshold & 0x0f) << 8;
		isl->isl29030_high_zone_info[i].upper_threshold =
			(als_high_lux.als_high_low_threshold & 0xf0) >> 4;
		isl->isl29030_high_zone_info[i].upper_threshold |=
			als_high_lux.als_high_threshold << 4;
		pr_err("%s:Element %i Upper 0x%X Lower 0x%X\n", __func__, i,
			isl->isl29030_high_zone_info[i].upper_threshold,
			isl->isl29030_high_zone_info[i].lower_threshold);
	}
}
static int ld_isl29030_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct isl29030_platform_data *pdata = client->dev.platform_data;
	struct isl29030_data *als_ir_data;
	int error = 0;

	if (pdata == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	} else if (!client->irq) {
		pr_err("%s: polling mode currently not supported\n", __func__);
		return -ENODEV;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C_FUNC_I2C not supported\n", __func__);
		return -ENODEV;
	}

	als_ir_data = kzalloc(sizeof(struct isl29030_data), GFP_KERNEL);
	if (als_ir_data == NULL) {
		error = -ENOMEM;
		goto err_alloc_data_failed;
	}

	als_ir_data->client = client;
	als_ir_data->als_ir_pdata = pdata;

	als_ir_data->idev = input_allocate_device();
	if (!als_ir_data->idev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
		       error);
		goto error_input_allocate_failed_ir;
	}

	als_ir_data->idev->name = "proximity";
	input_set_capability(als_ir_data->idev, EV_ABS, ABS_DISTANCE);

	als_ir_data->adev = input_allocate_device();
	if (!als_ir_data->adev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
		       error);
		goto error_input_allocate_failed_als;
	}

	als_ir_data->adev->name = "als";
	input_set_capability(als_ir_data->adev, EV_MSC, MSC_RAW);
	input_set_capability(als_ir_data->adev, EV_LED, LED_MISC);

	error = misc_register(&isl29030_misc_device);
	if (error < 0) {
		pr_err("%s: isl29030 register failed\n", __func__);
		goto error_misc_register_failed;
	}

	als_ir_data->prox_detect = 0;
	als_ir_data->low_lux_level = 0;
	isl29030_convert_zones(als_ir_data);

	atomic_set(&als_ir_data->enabled, 0);

	als_ir_data->working_queue = create_singlethread_workqueue("als_wq");
	if (!als_ir_data->working_queue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		error = -ENOMEM;
		goto error_create_wq_failed;
	}

	INIT_WORK(&als_ir_data->wq, ld_isl29030_work_queue);

	error = request_irq(als_ir_data->client->irq, ld_isl29030_irq_handler,
			    (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
				LD_ISL29030_NAME, als_ir_data);
	if (error != 0) {
		pr_err("%s: irq request failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_req_irq_failed;
	}
	enable_irq_wake(als_ir_data->client->irq);

	i2c_set_clientdata(client, als_ir_data);

	error = input_register_device(als_ir_data->idev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
		       error);
		goto error_input_register_failed_ir;
	}

	error = input_register_device(als_ir_data->adev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
		       error);
		goto error_input_register_failed_als;
	}

	error = ld_isl29030_init_registers(als_ir_data);
	if (error < 0) {
		pr_err("%s: Register Initialization failed: %d\n",
		       __func__, error);
		error = -ENODEV;
		goto err_reg_init_failed;
	}

	error = device_create_file(&als_ir_data->client->dev,
		&dev_attr_registers);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_create_registers_file_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	als_ir_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	als_ir_data->early_suspend.suspend = isl29030_early_suspend;
	als_ir_data->early_suspend.resume = isl29030_late_resume;
	register_early_suspend(&als_ir_data->early_suspend);
#endif
	disable_irq(als_ir_data->client->irq);
	queue_work(als_ir_data->working_queue, &als_ir_data->wq);

	return 0;

err_create_registers_file_failed:
err_reg_init_failed:
    input_unregister_device(als_ir_data->adev);
error_input_register_failed_als:
	input_unregister_device(als_ir_data->idev);
error_input_register_failed_ir:
	free_irq(als_ir_data->client->irq, als_ir_data);
err_req_irq_failed:
	destroy_workqueue(als_ir_data->working_queue);
error_misc_register_failed:
error_create_wq_failed:
	input_free_device(als_ir_data->adev);
error_input_allocate_failed_als:
	input_free_device(als_ir_data->idev);
error_input_allocate_failed_ir:
	kfree(als_ir_data);
err_alloc_data_failed:
	return error;
}

static int ld_isl29030_remove(struct i2c_client *client)
{
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);

	input_free_device(als_ir_data->idev);
	input_free_device(als_ir_data->adev);
	device_remove_file(als_ir_data->led_dev.dev, &dev_attr_registers);

	free_irq(als_ir_data->client->irq, als_ir_data);

	if (als_ir_data->working_queue)
		destroy_workqueue(als_ir_data->working_queue);
	kfree(als_ir_data);
	return 0;
}

static int isl29030_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);
	int ret;
	u8 config_data = 0;

	if (isl29030_debug)
		pr_info("%s: Suspending\n", __func__);

	if (atomic_read(&als_ir_data->enabled) == 1) {
		/* Disable the ALS but not the IRQ because the prox
		is enabled */
		ret = isl29030_read_reg(als_ir_data, ISL29030_CONFIGURE,
			&config_data);
		if (ret != 0) {
			pr_err("%s:Unable to read interrupt register: %d\n",
		       __func__, ret);
		}
		ret = isl29030_write_reg(als_ir_data, ISL29030_CONFIGURE,
			     config_data & 0xfb);
		if (ret)
			pr_err("%s:Unable to turn on prox: %d\n",
		       __func__, ret);
	} else {
		disable_irq_nosync(als_ir_data->client->irq);
		ret = cancel_work_sync(&als_ir_data->wq);
		if (ret) {
			pr_info("%s: Not Suspending\n", __func__);
			enable_irq(als_ir_data->client->irq);
			return -EBUSY;
		}
	}

	return 0;
}

static int isl29030_resume(struct i2c_client *client)
{
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);
	int ret;
	u8 config_data = 0;

	if (isl29030_debug)
		pr_info("%s: Resuming\n", __func__);

	/* Disable the ALS but not the IRQ because the prox
	is enabled */
	ret = isl29030_read_reg(als_ir_data, ISL29030_CONFIGURE, &config_data);
	if (ret != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
	       __func__, ret);
	}
	ret = isl29030_write_reg(als_ir_data, ISL29030_CONFIGURE,
		     config_data | 0x04);
	if (ret)
		pr_err("%s:Unable to turn on prox: %d\n",
	       __func__, ret);

	/* Allow the ALS sensor to read the zone */
	msleep(100);
	isl29030_report_input(als_ir_data);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29030_early_suspend(struct early_suspend *handler)
{
	struct isl29030_data *als_ir_data;

	als_ir_data = container_of(handler, struct isl29030_data,
			early_suspend);
	isl29030_suspend(als_ir_data->client, PMSG_SUSPEND);
}

static void isl29030_late_resume(struct early_suspend *handler)
{
	struct isl29030_data *als_ir_data;

	als_ir_data = container_of(handler, struct isl29030_data,
			early_suspend);
	isl29030_resume(als_ir_data->client);
}
#endif

static const struct i2c_device_id isl29030_id[] = {
	{LD_ISL29030_NAME, 0},
	{}
};

static struct i2c_driver ld_isl29030_i2c_driver = {
	.probe = ld_isl29030_probe,
	.remove = ld_isl29030_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= isl29030_suspend,
	.resume		= isl29030_resume,
#endif
	.id_table = isl29030_id,
	.driver = {
		   .name = LD_ISL29030_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init ld_isl29030_init(void)
{
	return i2c_add_driver(&ld_isl29030_i2c_driver);
}

static void __exit ld_isl29030_exit(void)
{
	i2c_del_driver(&ld_isl29030_i2c_driver);

}

module_init(ld_isl29030_init);
module_exit(ld_isl29030_exit);

MODULE_DESCRIPTION("ALS and Proximity driver for ISL29030");
MODULE_AUTHOR("Dan Murphy <wldm10@motorola.com>");
MODULE_LICENSE("GPL");
