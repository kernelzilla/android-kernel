/*
 * drivers/i2c/chips/lp3907_i2c.c
 *
 * I2C slave driver for LP3907 device
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Revision History:
 *
 * Date          Author    Comment
 * -----------   --------  -------------------
 * Jun-16-2008   Motorola  Initial version
 * Jan-04-2009   Motorola  Deleted atmega init irq
 */

/* enable/disable printk message */
/*#define DEBUG*/
#undef DEBUG

#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>		/* everything... */
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <asm/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "linux/i2c/lp3907_i2c.h"

#define LP3907_REG_ADDR_LEN    (1)    /* 1 BYTE */
#define I2C_RETRIES		5
#define I2C_RETRY_DELAY		5

/* LP3907 control register map */

/* LP3907 POWER STATUS */
#define LP3907_INIT_STATUS	0
#define LP3907_PWRUP_STATUS	1
#define LP3907_PWRDOWN_STATUS 	2

typedef enum {
	LP3907_ICRA_REG		= 0x02,
	LP3907_SCR1_REG         = 0x07,
	LP3907_BKLDOEN_REG      = 0x10,
	LP3907_BKLDOSR_REG      = 0x11,
	LP3907_VCCR_REG         = 0x20,
	LP3907_B1TV1_REG        = 0x23,
	LP3907_B1TV2_REG        = 0x24,
	LP3907_B1RC_REG         = 0x25,
	LP3907_B2TV1_REG        = 0x29,
	LP3907_B2TV2_REG        = 0x2A,
	LP3907_B2RC_REG         = 0x2B,
	LP3907_BFCR_REG         = 0x38,
	LP3907_LDO1VCR_REG      = 0x39,
	LP3907_LDO2VCR_REG      = 0x3A,
	LP3907_REG_MAX          = LP3907_LDO2VCR_REG,
} LP3907_REG;

#define LP3907_INVALID_REG      (LP3907_REG_MAX+1)

typedef struct {
	LP3907_REG reg;
	u8	   data;
} LP3907_cfg;
/* */

static LP3907_cfg lp3907_init_tbl[] = {
	{LP3907_SCR1_REG,    0x28,},
	{LP3907_BKLDOEN_REG, 0x74,},
	{LP3907_VCCR_REG,    0x00,},
	{LP3907_B2TV1_REG,   0x09,},  /* BUCK2 - 1.8V */
	{LP3907_B2RC_REG,    0x28,},
	{LP3907_BFCR_REG,    0x04,},
	{LP3907_LDO1VCR_REG, 0x02,},  /* LDO1 - 1.2V */
	{LP3907_LDO2VCR_REG, 0x08,},  /* LDO2 - 1.8V */

	/* must end invalid register */
	{LP3907_INVALID_REG, 0x00,},
};

static LP3907_cfg lp3907_pwr_off_tbl[] = {
	{LP3907_BKLDOEN_REG, 0x70,},    /* BUCK1 disable */
	{LP3907_BKLDOEN_REG, 0x60,},    /* LDO1EN disable */
	{LP3907_BKLDOEN_REG, 0x20,},    /* LDO2EN disable */

	/* must end invalid register */
	{LP3907_INVALID_REG, 0x00,},
};

struct lp3907_data {
	struct i2c_client *client;
	struct lp3907_platform_data *pdata;

	struct mutex lock; /* lock lp3907 */

	int mode;
	atomic_t enabled;
};

struct lp3907_data *g_lp3907_data;

static struct i2c_driver lp3907_driver;
static struct class *lp3907_class;
static struct cdev lp3907_cdev;

/* device major number */
int lp3907_major_num = 240;
int lp3907_minor_num;

/* *****************************************************

	Low level interface routine.

* *****************************************************/

/* write only 1 byte */
static int lp3907_i2c_write(struct lp3907_data *pLP3907, u8 reg_addr, u8 value)
{
	struct i2c_msg msg;
	u8 buf[sizeof(u8) + sizeof(u8)];
	int ret = 0, try_cnt = 0;

	buf[0] = reg_addr;
	memcpy((void *)&buf[LP3907_REG_ADDR_LEN], (void *)&value, sizeof(u8));

	msg.addr  = pLP3907->client->addr;
	msg.flags = 0;   /* I2C_M_WR write the register value */
	msg.buf   = buf;
	msg.len   = sizeof(buf);

	do {
		/* one messages */
		ret = i2c_transfer(pLP3907->client->adapter, &msg, 1);

		if (ret != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 1) && ((++try_cnt) < I2C_RETRIES));

	if (ret != 1) {
		dev_err(&pLP3907->client->dev, "[LP3907]write transfer error\n");
		ret = -EIO;
	} else
		ret = 0;

	return ret;
}

static int lp3907_i2c_read(struct lp3907_data *pLP3907, u8 reg_addr, u8 *buf)
{
	struct i2c_msg msg;
	u8 value[sizeof(u8) + sizeof(u8)];
	int ret = 0, try_cnt = 0;

	value[0] = reg_addr;
	memset((void *)&value[LP3907_REG_ADDR_LEN], 0, sizeof(u8));

	/* */
	msg.addr  = pLP3907->client->addr;
	msg.flags = I2C_M_RD;
	msg.buf   = buf;
	msg.len = sizeof(u8);

	do {
		ret = i2c_transfer(pLP3907->client->adapter, &msg, 1);
		if (ret != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 1) && (++try_cnt < I2C_RETRIES));

	if (ret != 1) {
		dev_err(&pLP3907->client->dev, "[LP3907] read transfer error\n");
		ret = -EIO;
	} else {
		memcpy((void *)buf, value + LP3907_REG_ADDR_LEN, sizeof(u8));
		ret = 0;
	}

	return ret;
}

static int lp3907_power_on_sequence(struct lp3907_data *pLP3907)
{
	int ret = 0;
	int i = 0;
	int line = 0;
	u8 ldo_status = 0x00;

	dev_info(&pLP3907->client->dev, "%s is called.\n", __func__);

	/* 1. set voltage regulator level. */
	for (; lp3907_init_tbl[i].reg != LP3907_INVALID_REG; i++) {
		if (ret == 0) {
			ret = lp3907_i2c_write(pLP3907, \
					       lp3907_init_tbl[i].reg, \
					       lp3907_init_tbl[i].data);
			if (!ret) {
				dev_dbg(&pLP3907->client->dev,
					"[LP3907]WR: reg = 0x%x, data = 0x%x\n",
					lp3907_init_tbl[i].reg,
					lp3907_init_tbl[i].data);
			}
		} else {
			dev_err(&pLP3907->client->dev,
				"lp3907_i2c_write(0x%x, 0x%x) is failed!\n",
				lp3907_init_tbl[i].reg,
				lp3907_init_tbl[i].data);
			line = __LINE__;
			goto lp3907_configure_exit;
		}
	}

	/* MDTV_REG pin control */
	if (pLP3907->pdata->power_on)
		pLP3907->pdata->power_on();

	/* 3. read status register */
	ret = lp3907_i2c_read(pLP3907, LP3907_BKLDOSR_REG, &ldo_status);
	if (!ret) {
		dev_err(&pLP3907->client->dev, "LP3907 read status = %d\n", ldo_status);
	}

	return 0;

lp3907_configure_exit:

	dev_err(&pLP3907->client->dev, "LP3907-CONFIGURE: failed [%d] line=[%d]\n", ret, line);
	return ret;
}

static int lp3907_power_off_sequence(struct lp3907_data *pLP3907)
{
	int i = 0;
	int line = 0, ret = 0;

	dev_info(&pLP3907->client->dev, "%s is called.\n", __func__);

	/* MDTV_REG pin control */
	if (pLP3907->pdata->power_off)
		pLP3907->pdata->power_off();

	/* LDO and BUCK disable */
	for (; lp3907_pwr_off_tbl[i].reg != LP3907_INVALID_REG; i++) {
		if (ret == 0) {
			ret = lp3907_i2c_write(pLP3907, \
					       lp3907_pwr_off_tbl[i].reg, \
					       lp3907_pwr_off_tbl[i].data);
			if (!ret) {
				dev_dbg(&pLP3907->client->dev,
				"[LP3907]WR: reg = 0x%x, data = 0x%x\n",
						lp3907_init_tbl[i].reg, lp3907_init_tbl[i].data);
			}
		} else {
			dev_err(&pLP3907->client->dev, "lp3907_i2c_write(%x, %x) is failed!\n",
			       lp3907_init_tbl[i].reg, lp3907_init_tbl[i].data);
			line = __LINE__;
			goto lp3907_poweroff_exit;
		}
	}

	return 0;

lp3907_poweroff_exit:

	dev_err(&pLP3907->client->dev, "LP3907-CONFIGURE: failed [%d] line=[%d]\n", ret, line);
	return ret;
}

static int lp3907_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	file->private_data = g_lp3907_data;

	return 0;
}

static int lp3907_release(struct inode *inode, struct file *file)
{
  return 0;
}

static int lp3907_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	/*void __user *argp = (void __user *)arg;*/
	int status = -1;
	struct lp3907_data *pLP3907 = file->private_data;

	switch (cmd) {
	case LP3907_PWR_OFF_CMD:
		/* power off sequence */
		status = lp3907_power_off_sequence(pLP3907);
		break;

	case LP3907_PWR_ON_CMD:
		/* power on sequence */
		status = lp3907_power_on_sequence(pLP3907);
		break;

	default:
		return -EINVAL;
	}

	return status;
}

struct file_operations lp3907_fops = {
	.owner   = THIS_MODULE,
	.open    = lp3907_open,
	.release = lp3907_release,
	.ioctl   = lp3907_ioctl,
};

/* *****************************************************

	Probe, Remove

* ******************************************************/
static int lp3907_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lp3907_data *pLP3907;
	int ret = -1, line;

	dev_info(&client->dev, "lp3907_probe() is called.\n");

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "[lp3907]platform data is NULL. exiting.\n");
		ret = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret  = -ENODEV;
		goto err0;
	}

	pLP3907 = kzalloc(sizeof(*pLP3907), GFP_KERNEL);
	if (pLP3907 == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err0;
	}

	mutex_init(&pLP3907->lock);
	mutex_lock(&pLP3907->lock);
	pLP3907->client = client;

	pLP3907->pdata = kmalloc(sizeof(*pLP3907->pdata), GFP_KERNEL);
	if (pLP3907->pdata == NULL)
		goto err1;
	memcpy(pLP3907->pdata, client->dev.platform_data, sizeof(*pLP3907->pdata));

	strncpy(client->name, LP3907_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, pLP3907);

	if (pLP3907->pdata->init) {
		ret = pLP3907->pdata->init();
		if (ret < 0)
			goto err1;
	}

    /* store platform data*/
	g_lp3907_data = pLP3907;

	ret = lp3907_power_off_sequence(pLP3907);
	if (ret < 0) {
		dev_err(&pLP3907->client->dev, "LP3907 chip configure is failed!\n");
		goto err2;
	}

	mutex_unlock(&pLP3907->lock);
	return ret;

err2:
	if (pLP3907->pdata->exit)
		pLP3907->pdata->exit();

err1:
	mutex_unlock(&pLP3907->lock);
	kfree(pLP3907->pdata);

err0:
	return ret;

}

static int __devexit lp3907_remove(struct i2c_client *client)
{
	struct lp3907_data *pLP3907 = i2c_get_clientdata(client);

	dev_info(&pLP3907->client->dev, "lp3907_remove() is called!\n");

	if (pLP3907->pdata->exit)
		pLP3907->pdata->exit();
	kfree(pLP3907->pdata);
	kfree(pLP3907);

	return 0;
}

static int lp3907_resume(struct i2c_client *client)
{
	struct lp3907_data *pLP3907 = i2c_get_clientdata(client);

	dev_info(&pLP3907->client->dev, "lp3907_resume() is called!\n");
	return 0;
}

static int lp3907_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lp3907_data *pLP3907 = i2c_get_clientdata(client);

	dev_info(&pLP3907->client->dev, "lp3907_suspend() is called!\n");
	return 0;
}

static const struct i2c_device_id lp3907_id[] = {
	{ LP3907_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp3907_id);

static struct i2c_driver lp3907_driver = {
	.driver = {
		.name = LP3907_DRIVER_NAME,
	},
	.probe	  = lp3907_probe,
	.remove	  = __devexit_p(lp3907_remove),
	.resume   = lp3907_resume,
	.suspend  = lp3907_suspend,
	.id_table = lp3907_id,
};

/*
 *  LP3907 voltage regulator init
 */
static int __init lp3907_init(void)
{
	struct device *lp3907_device;
	dev_t devno;

	int ret = -1;
	int line;

	devno = MKDEV(lp3907_major_num, lp3907_minor_num);
	ret = register_chrdev_region(devno, 1, "lp3907");
	if (ret < 0) {
		printk(KERN_ERR "[FATAL] LP3907 device is failed.\n");
		return ret;
	}

	lp3907_class = class_create(THIS_MODULE, "lp3907");
	if (IS_ERR(lp3907_class)) {
		unregister_chrdev_region(devno, 1);
		ret = -EFAULT;
	} else {

	lp3907_device = device_create(lp3907_class,
			NULL,
			MKDEV(lp3907_major_num, lp3907_minor_num),
			NULL,
			"lp3907");
		if (IS_ERR(lp3907_device)) {
			class_destroy(lp3907_class);
			unregister_chrdev_region(devno, 1);
			ret = -EFAULT;
		} else {

	cdev_init(&lp3907_cdev, &lp3907_fops);
	lp3907_cdev.owner = THIS_MODULE;
	lp3907_cdev.ops   = &lp3907_fops;
	kobject_set_name(&lp3907_cdev.kobj, "lp3907");
	cdev_add(&lp3907_cdev, MKDEV(lp3907_major_num, lp3907_minor_num), 1);

	ret = i2c_add_driver(&lp3907_driver);
	if (ret) {
		line = __LINE__;
		goto init_exit_path;
	}
    }
	}

	return ret;
init_exit_path:
	printk(KERN_ERR "[FATAL] Unable to register LP3907 i2c driver!\n");
	i2c_del_driver(&lp3907_driver);
	return ret;
}

static void __exit lp3907_exit(void)
{
	i2c_del_driver(&lp3907_driver);

	class_destroy(lp3907_class);
	unregister_chrdev_region(MKDEV(lp3907_major_num, lp3907_minor_num), 1);

	cdev_del(&lp3907_cdev);
	printk(KERN_INFO "LP3907 voltage regulator driver: exit\n");
}

module_init(lp3907_init);
module_exit(lp3907_exit);


MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("LP3907 voltage regulator driver");
MODULE_LICENSE("GPL");

