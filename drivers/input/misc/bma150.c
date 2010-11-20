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
/*
 * Protocol driver for Bosch BMA150 accelerometer
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/bma150.h>

#define BMA150_CMD_READ                  0x80
#define BMA150_REG_CHIPID                0x00
#define BMA150_REG_ACCX_LS               0x02
#define BMA150_REG_CONTROL_0A            0x0A
#define BMA150_REG_CONTROL_0B            0x0B
#define BMA150_REG_ANY_MOTION_THRESH     0x10
#define BMA150_REG_WIDTH_BANDW           0x14
#define BMA150_REG_CONTROL_15            0x15
#define BMA150_LAST_REG                  0x15

#define BMA150_REG_C0A_RESET_INT         0x40
#define BMA150_REG_C0A_SLEEP             0x01

#define BMA150_REG_C0B_ANY_MOTION        0x40
#define BMA150_REG_C0B_ENABLE_HG         0x02
#define BMA150_REG_C0B_ENABLE_LG         0x01

#define BMA150_REG_WID_BANDW_MASK        0x07

#define BMA150_REG_C15_SPI4              0x80
#define BMA150_REG_C15_EN_ADV_INT        0x40
#define BMA150_REG_C15_NEW_DATA_INT      0x20
#define BMA150_REG_C15_LATCH_INT         0x10

#define BMA150_BANDW_INIT                0x04
#define BMA150_ANY_MOTION_INIT           0x02

/* temperature offset of -30 degrees in units of 0.5 degrees */
#define BMA150_TEMP_OFFSET               60

#define BMA150_NAME                      "bma150"
#define BMA150_NAME_FORMAT               "bma150-%d.%d"
#define BMA150_DEVICE_NAME               "/dev/bma150-%d.%d"
#define BMA150_VENDORID                  0x0001

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("bma150");

/*
 * Data returned from accelerometer.
 * Temp is in units of 0.5 degrees C
 */
struct bma150_accel_data {
	int              accel_x;
	int              accel_y;
	int              accel_z;
	int              temp;
};

struct driver_data {
	struct input_dev         *ip_dev;
	struct spi_device        *spi;
	char                      rx_buf[16];
	char                      bits_per_transfer;
	struct work_struct        work_data;
	bool                      config;
	struct list_head          next_dd;
	struct dentry            *dbfs_root;
	struct dentry            *dbfs_bpw;
	struct dentry            *dbfs_regs;
};

static struct mutex               bma150_dd_lock;
static struct list_head           dd_list;

#if defined(CONFIG_DEBUG_FS)
static int bma150_dbfs_open(struct inode *inode, struct file *fp)
{
	fp->private_data = inode->i_private;
	return 0;
}

static ssize_t bma150_dbfs_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *f_pos)
{
	u8                          *p;
	u8                          *np;
	u8                          *tx_buf;
	u8                          *mbuf;
	int                          i;
	int                          rc;
	int                          max_num_writes;
	unsigned int                 val;
	struct spi_message           m;
	struct driver_data          *dd;
	struct spi_transfer          t;

	/* format of write data is "A[A] D[D]" eg. "AA DD", "A D" etc
	   where A is address in hex, D is data in hex.
	   Multiple address/data pairs may be separated by spaces.
	*/
	if (count < 3)
		return 0;

	max_num_writes = (count + 1) / 4;
	dd = fp->private_data;

	mbuf = kzalloc(count, GFP_KERNEL);
	if (!mbuf) {
		rc = -ENOMEM;
		goto dbfs_write_exit;
	}

	tx_buf = kzalloc(max_num_writes * 2, GFP_KERNEL);
	if (!tx_buf) {
		rc = -ENOMEM;
		goto dbfs_write_exit_txbuf;
	}

	if (copy_from_user(mbuf, buf, count)) {
		rc = -EFAULT;
		goto dbfs_write_exit_copy;
	}

	memset(&t, 0, sizeof t);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	t.tx_buf = tx_buf;
	t.rx_buf = NULL;
	p = mbuf;

	while (isspace(*p))
		p++;
	i = 0;
	do {
		val = simple_strtoul(p, (char **)&np, 16);
		if ((val > BMA150_LAST_REG) || (p == np)) {
			rc = -EINVAL;
			goto dbfs_write_exit_copy;
		}
		while (isspace(*np) && ((np - mbuf) < count))
			np++;
		p = np;
		tx_buf[i*2] = (u8)val;

		val = simple_strtoul(p, (char **)&np, 16);
		if ((val > 0xFF)  || (p == np)) {
			rc = -EINVAL;
			goto dbfs_write_exit_copy;
		}
		while (isspace(*np) && ((np - mbuf) < count))
			np++;
		p = np;
		tx_buf[i * 2 + 1] = (u8)val;
		i++;
	} while ((np - mbuf) < count);

	t.len = i * 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto dbfs_write_exit;
	kfree(mbuf);
	kfree(tx_buf);

	return count;

dbfs_write_exit_copy:
	kfree(tx_buf);
dbfs_write_exit_txbuf:
	kfree(mbuf);
dbfs_write_exit:
	return rc;
}

static ssize_t bma150_dbfs_read(struct file *fp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	u8                           tx_buf[2];
	u8                           rx_buf[2];
	u8                           mbuf[8];
	int                          rc;
	int                          copy_size;
	struct spi_message           m;
	struct spi_transfer          t;
	struct driver_data          *dd;

	dd = fp->private_data;
	if ((int)*f_pos > BMA150_LAST_REG) {
		rc = 0;
		goto dbfs_read_exit;
	}

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 2;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	tx_buf[0] = BMA150_CMD_READ | (u8)*f_pos;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto dbfs_read_exit;

	snprintf(mbuf, ARRAY_SIZE(mbuf), "%02x ", rx_buf[1]);
	copy_size = min(count, strlen(mbuf) + 1);
	if (copy_to_user(buf, mbuf, count))
		return -EFAULT;
	(*f_pos)++;

	return copy_size;
dbfs_read_exit:
	return rc;
}

static const struct file_operations dbfs_fops = {
	.owner    = THIS_MODULE,
	.open     = bma150_dbfs_open,
	.read     = bma150_dbfs_read,
	.write    = bma150_dbfs_write,
};

static ssize_t bma150_dbfs_bpw_write(struct file *fp, const char __user *buf,
				     size_t count, loff_t *f_pos)
{
	unsigned long                new_bpw;
	int                          rc;
	int                          copy_len;
	char                         mbuf[16];
	struct driver_data          *dd;

	dd = fp->private_data;
	copy_len = min(count, ARRAY_SIZE(mbuf) - 1);
	if (copy_from_user(mbuf, buf, copy_len)) {
		rc = -EFAULT;
		goto dbfs_bpw_write_exit;
	}
	mbuf[copy_len] = '\0';
	rc = strict_strtoul(mbuf, 10, &new_bpw);
	if (rc)
		goto dbfs_bpw_write_exit;

	if ((new_bpw < 1) || (new_bpw > 4)) {
		rc = -EINVAL;
		goto dbfs_bpw_write_exit;
	}

	dd->spi->bits_per_word = (new_bpw * BITS_PER_BYTE) & 0xff;
	return copy_len;

dbfs_bpw_write_exit:
	return rc;
}

static ssize_t bma150_dbfs_bpw_read(struct file *fp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	char                         mbuf[16];
	struct driver_data          *dd;

	dd = fp->private_data;
	snprintf(mbuf, ARRAY_SIZE(mbuf), "%d",
		 dd->spi->bits_per_word / BITS_PER_BYTE);
	return simple_read_from_buffer(buf, count, f_pos,
				       mbuf, strlen(mbuf) + 1);
}

static const struct file_operations dbfs_bpw_fops = {
	.owner    = THIS_MODULE,
	.open     = bma150_dbfs_open,
	.read     = bma150_dbfs_bpw_read,
	.write    = bma150_dbfs_bpw_write,
};

static void __devinit bma150_create_dbfs_entry(struct driver_data *dd)
{
	char buf[16];

	snprintf(buf, sizeof(buf), BMA150_NAME_FORMAT,
		 dd->spi->master->bus_num, dd->spi->chip_select);
	dd->dbfs_root = debugfs_create_dir(buf, NULL);
	if (dd->dbfs_root <= (struct dentry *)NULL) {
		dd->dbfs_root = NULL;
		goto dbfs_err_root;
	}
	dd->dbfs_bpw = debugfs_create_file("bytes_per_word",
					   S_IRUGO | S_IWUGO,
					   dd->dbfs_root, dd,
					   &dbfs_bpw_fops);
	if (dd->dbfs_bpw <= (struct dentry *)NULL) {
		dd->dbfs_bpw = NULL;
		goto dbfs_err_bpw;
	}

	dd->dbfs_regs = debugfs_create_file("registers",
					   S_IRUGO | S_IWUGO,
					   dd->dbfs_root, dd,
					   &dbfs_fops);
	if (dd->dbfs_regs <= (struct dentry *)NULL) {
		dd->dbfs_regs = NULL;
		goto dbfs_err_regs;
	}
	return;

dbfs_err_regs:
	debugfs_remove(dd->dbfs_bpw);
dbfs_err_bpw:
	debugfs_remove(dd->dbfs_root);
dbfs_err_root:
	return;
}

static void __devexit bma150_remove_dbfs_entry(struct driver_data *dd)
{
	if (dd->dbfs_regs)
		debugfs_remove(dd->dbfs_regs);
	if (dd->dbfs_bpw)
		debugfs_remove(dd->dbfs_bpw);
	if (dd->dbfs_root)
		debugfs_remove(dd->dbfs_root);
}
#else
static void __devinit bma150_create_dbfs_entry(struct driver_data *dd) { }

static void __devexit bma150_remove_dbfs_entry(struct driver_data *dd) { }
#endif

#ifdef CONFIG_PM
static int bma150_suspend(struct spi_device *spi, pm_message_t mesg)
{
	disable_irq(spi->irq);
	return 0;
}

static int bma150_resume(struct spi_device *spi)
{
	enable_irq(spi->irq);
	return 0;
}
#else
#define bma150_suspend NULL
#define bma150_resume NULL
#endif /* CONFIG_PM */

static irqreturn_t bma150_irq(int irq, void *dev_id)
{
	struct device      *dev = dev_id;
	struct driver_data *dd;

	dd = dev_get_drvdata(dev);
	schedule_work(&dd->work_data);
	return IRQ_HANDLED;
}

static int bma150_open(struct input_dev *dev)
{
	int                 rc = 0;
	struct driver_data *dd = input_get_drvdata(dev);

	if (!dd->spi->irq)
		return -1;

	rc = request_irq(dd->spi->irq,
			 &bma150_irq,
			 IRQF_TRIGGER_RISING,
			 BMA150_NAME,
			 &dd->spi->dev);
	return rc;
}

static void bma150_release(struct input_dev *dev)
{
	struct driver_data *dd = input_get_drvdata(dev);

	free_irq(dd->spi->irq, &dd->spi->dev);
}

static void convert_regdata_to_accel_data(u8 *buf, struct bma150_accel_data *a)
{
	/* The BMA150 returns 10-bit values split over 2 bytes */
	a->accel_x = ((buf[1] & 0xC0) >> 6) | (buf[2] << 2);
	a->accel_y = ((buf[3] & 0xC0) >> 6) | (buf[4] << 2);
	a->accel_z = ((buf[5] & 0xC0) >> 6) | (buf[6] << 2);
	/* convert 10-bit signed value to 32-bit */
	if (a->accel_x & 0x200)
		a->accel_x = a->accel_x - 0x400;
	if (a->accel_y & 0x200)
		a->accel_y = a->accel_y - 0x400;
	if (a->accel_z & 0x200)
		a->accel_z = a->accel_z - 0x400;
	/* 0-based, units are 0.5 degree C */
	a->temp = buf[7] - BMA150_TEMP_OFFSET;
}

static void bma150_work_f(struct work_struct *work)
{
	u8                          tx_buf[8];
	u8                          rx_buf[8];
	int                         rc;
	int                         i;
	struct spi_message          m;
	struct spi_transfer         t;
	struct driver_data         *dd =
		container_of(work, struct driver_data, work_data);
	struct bma150_accel_data    acc_data;

	tx_buf[0] = BMA150_CMD_READ | BMA150_REG_ACCX_LS;
	for (i = 1; i < 8; i++)
		tx_buf[i] = 0x00;
	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 8;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto workf_exit;

	convert_regdata_to_accel_data(t.rx_buf, &acc_data);
	input_report_rel(dd->ip_dev, REL_X, acc_data.accel_x);
	input_report_rel(dd->ip_dev, REL_Y, acc_data.accel_y);
	input_report_rel(dd->ip_dev, REL_Z, acc_data.accel_z);
	input_report_rel(dd->ip_dev, REL_MISC, acc_data.temp);
	input_sync(dd->ip_dev);

	tx_buf[0] = BMA150_REG_CONTROL_0A;
	tx_buf[1] = BMA150_REG_C0A_RESET_INT;
	t.rx_buf = NULL;
	t.len = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto workf_exit;

	return;

workf_exit:
	dev_err(&dd->ip_dev->dev, "%s: exit with error %d\n", __func__, rc);
}

static int __devexit bma150_power_down(struct driver_data *dd)
{
	char                tx_buf[2];
	int                 rc;
	struct spi_message  m;
	struct spi_transfer t;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	spi_setup(dd->spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = BMA150_REG_CONTROL_0A;
	tx_buf[1] = BMA150_REG_C0A_SLEEP;
	t.rx_buf = NULL;
	t.len = 2;
	rc = spi_sync(dd->spi, &m);
	return rc;
}

static int __devinit bma150_power_up(struct driver_data *dd)
{
	char                tx_buf[8];
	int                 rc;
	struct spi_message  m;
	struct spi_transfer t;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	spi_setup(dd->spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = BMA150_REG_CONTROL_0A;
	tx_buf[1] = 0x00;
	t.rx_buf = NULL;
	t.len = 2;
	rc = spi_sync(dd->spi, &m);
	return rc;
}

static int __devinit bma150_config(struct driver_data *dd)
{
	char                tx_buf[8];
	char                rx_buf[8];
	int                 rc;
	struct spi_message  m;
	struct spi_transfer t;
	u8                  reg_bandw;
	u8                  reg_15;

	dd->spi->bits_per_word = 32;
	bma150_power_up(dd);
	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	spi_setup(dd->spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = BMA150_CMD_READ | BMA150_REG_CHIPID;
	t.len = 3;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	if ((rx_buf[1] == 0x00) || (rx_buf[2] == 0x00)) {
		printk(KERN_ERR "bma150 accelerometer not detected\n");
		rc = -ENODEV;
		goto config_exit;
	}
	printk(KERN_INFO "bma150: detected chip id %d\n", rx_buf[1] & 0x07);

	tx_buf[0] = BMA150_CMD_READ | BMA150_REG_WIDTH_BANDW;
	t.len = 3;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;

	reg_bandw = rx_buf[1];
	reg_15    = rx_buf[2];

	tx_buf[0] = BMA150_REG_CONTROL_15;
	tx_buf[1] = reg_15 | BMA150_REG_C15_EN_ADV_INT |
		BMA150_REG_C15_LATCH_INT;
	tx_buf[2] = BMA150_REG_CONTROL_0B;
	tx_buf[3] = BMA150_REG_C0B_ANY_MOTION |
		BMA150_REG_C0B_ENABLE_HG  |
		BMA150_REG_C0B_ENABLE_LG;
	tx_buf[4] = BMA150_REG_ANY_MOTION_THRESH;
	tx_buf[5] = BMA150_ANY_MOTION_INIT;
	tx_buf[6] = BMA150_REG_WIDTH_BANDW;
	tx_buf[7] = (reg_bandw & ~BMA150_REG_WID_BANDW_MASK) |
		BMA150_BANDW_INIT;
	t.len = 8;
	t.rx_buf = NULL;
	rc = spi_sync(dd->spi, &m);

config_exit:
	return rc;
}

static int __devinit bma150_probe(struct spi_device *spi)
{
	struct driver_data *dd;
	int                 rc;
	char               *devname;
	struct bma150_platform_data *pdata = spi->dev.platform_data;

	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	devname = kzalloc(sizeof(BMA150_DEVICE_NAME) + 1, GFP_KERNEL);
	if (!devname) {
		rc = -ENOMEM;
		goto probe_exit_alloc;
	}

	mutex_lock(&bma150_dd_lock);
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&bma150_dd_lock);
	INIT_WORK(&dd->work_data, bma150_work_f);
	dd->spi = spi;
	rc = bma150_config(dd);
	if (rc)
		goto probe_err_cfg;

	if (pdata && pdata->setup) {
		rc = pdata->setup(&spi->dev);
		if (rc)
			goto probe_err_cfg;
	}

	bma150_create_dbfs_entry(dd);
	spi_set_drvdata(spi, dd);

	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	snprintf(devname, sizeof(BMA150_DEVICE_NAME) + 1, BMA150_DEVICE_NAME,
		 spi->master->bus_num, spi->chip_select);
	dd->ip_dev->open       = bma150_open;
	dd->ip_dev->close      = bma150_release;
	dd->ip_dev->name       = BMA150_NAME;
	dd->ip_dev->phys       = devname;
	dd->ip_dev->id.vendor  = BMA150_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
	__set_bit(EV_REL,    dd->ip_dev->evbit);
	__set_bit(REL_X,     dd->ip_dev->relbit);
	__set_bit(REL_Y,     dd->ip_dev->relbit);
	__set_bit(REL_Z,     dd->ip_dev->relbit);
	__set_bit(REL_MISC,  dd->ip_dev->relbit);
	rc = input_register_device(dd->ip_dev);
	if (rc) {
		dev_err(&dd->ip_dev->dev,
			"bma150_probe: input_register_device rc=%d\n",
		       rc);
		goto probe_err_reg_dev;
	}

	return rc;

probe_err_reg_dev:
	dd->ip_dev = NULL;
	input_free_device(dd->ip_dev);
probe_err_reg:
	bma150_remove_dbfs_entry(dd);
	spi_set_drvdata(spi, NULL);
	if (pdata && pdata->teardown)
		pdata->teardown(&spi->dev);
probe_err_cfg:
	mutex_lock(&bma150_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma150_dd_lock);
	kfree(devname);
probe_exit_alloc:
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit bma150_remove(struct spi_device *spi)
{
	struct driver_data *dd;
	int                 rc;
	const char	   *devname;
	struct bma150_platform_data *pdata = spi->dev.platform_data;

	dd = spi_get_drvdata(spi);
	devname = dd->ip_dev->phys;

	rc = bma150_power_down(dd);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: power down failed with error %d\n",
			__func__, rc);
	input_unregister_device(dd->ip_dev);
	bma150_remove_dbfs_entry(dd);
	spi_set_drvdata(spi, NULL);
	if (pdata && pdata->teardown)
		pdata->teardown(&spi->dev);
	mutex_lock(&bma150_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma150_dd_lock);
	kfree(devname);
	kfree(dd);

	return 0;
}

static struct spi_driver bma150_driver = {
	.driver = {
		.name  = BMA150_NAME,
		.owner = THIS_MODULE,
	},
	.probe         = bma150_probe,
	.remove        = __devexit_p(bma150_remove),
	.suspend       = bma150_suspend,
	.resume        = bma150_resume,
};


static int __init bma150_init(void)
{
	int rc;

	INIT_LIST_HEAD(&dd_list);
	mutex_init(&bma150_dd_lock);

	rc = spi_register_driver(&bma150_driver);
	return rc;
}
module_init(bma150_init);

static void __exit bma150_exit(void)
{
	spi_unregister_driver(&bma150_driver);
}
module_exit(bma150_exit);
