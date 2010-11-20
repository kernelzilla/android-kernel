/* drivers/i2c/busses/i2c-msm.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/mach-types.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/mutex.h>
#include <linux/remote_spinlock.h>
#include <linux/pm_qos_params.h>

#define DEBUG 0

enum {
	I2C_WRITE_DATA          = 0x00,
	I2C_CLK_CTL             = 0x04,
	I2C_STATUS              = 0x08,
	I2C_READ_DATA           = 0x0c,
	I2C_INTERFACE_SELECT    = 0x10,

	I2C_WRITE_DATA_DATA_BYTE            = 0xff,
	I2C_WRITE_DATA_ADDR_BYTE            = 1U << 8,
	I2C_WRITE_DATA_LAST_BYTE            = 1U << 9,

	I2C_CLK_CTL_FS_DIVIDER_VALUE        = 0xff,
	I2C_CLK_CTL_HS_DIVIDER_VALUE        = 7U << 8,

	I2C_STATUS_WR_BUFFER_FULL           = 1U << 0,
	I2C_STATUS_RD_BUFFER_FULL           = 1U << 1,
	I2C_STATUS_BUS_ERROR                = 1U << 2,
	I2C_STATUS_PACKET_NACKED            = 1U << 3,
	I2C_STATUS_ARB_LOST                 = 1U << 4,
	I2C_STATUS_INVALID_WRITE            = 1U << 5,
	I2C_STATUS_FAILED                   = 3U << 6,
	I2C_STATUS_BUS_ACTIVE               = 1U << 8,
	I2C_STATUS_BUS_MASTER               = 1U << 9,
	I2C_STATUS_ERROR_MASK               = 0xfc,

	I2C_INTERFACE_SELECT_INTF_SELECT    = 1U << 0,
	I2C_INTERFACE_SELECT_SCL            = 1U << 8,
	I2C_INTERFACE_SELECT_SDA            = 1U << 9,
	I2C_STATUS_RX_DATA_STATE            = 3U << 11,
};

struct msm_i2c_dev {
	struct device                *dev;
	void __iomem                 *base;	/* virtual */
	int                          irq;
	struct clk                   *clk;
#ifdef CONFIG_KERNEL_MOTOROLA
	int gpio_i2c_scl;
	int gpio_i2c_sda;
#endif /* CONFIG_MACH_CALGARY */
#ifndef CONFIG_MACH_MOT
	struct i2c_adapter           adap_pri;
	struct i2c_adapter           adap_aux;
#else
	struct i2c_adapter           adapter;
#endif

	spinlock_t                   lock;

	struct i2c_msg               *msg;
	int                          rem;
	int                          pos;
	int                          cnt;
	int                          err;
	int                          flush_cnt;
	int                          rd_acked;
	remote_spinlock_t            rspin_lock;
	int                          suspended;
	struct mutex                 mlock;
	struct msm_i2c_platform_data *pdata;
	void                         *complete;
};

#if DEBUG
static void
dump_status(uint32_t status)
{
	printk("STATUS (0x%.8x): ", status);
	if (status & I2C_STATUS_BUS_MASTER)
		printk("MST ");
	if (status & I2C_STATUS_BUS_ACTIVE)
		printk("ACT ");
	if (status & I2C_STATUS_INVALID_WRITE)
		printk("INV_WR ");
	if (status & I2C_STATUS_ARB_LOST)
		printk("ARB_LST ");
	if (status & I2C_STATUS_PACKET_NACKED)
		printk("NAK ");
	if (status & I2C_STATUS_BUS_ERROR)
		printk("BUS_ERR ");
	if (status & I2C_STATUS_RD_BUFFER_FULL)
		printk("RD_FULL ");
	if (status & I2C_STATUS_WR_BUFFER_FULL)
		printk("WR_FULL ");
	if (status & I2C_STATUS_FAILED)
		printk("FAIL 0x%x", (status & I2C_STATUS_FAILED));
	printk("\n");
}
#endif

static irqreturn_t
msm_i2c_interrupt(int irq, void *devid)
{
	struct msm_i2c_dev *dev = devid;
	uint32_t status = readl(dev->base + I2C_STATUS);
	int err = 0;

#if DEBUG
	dump_status(status);
#endif

	spin_lock(&dev->lock);
	if (!dev->msg) {
		printk(KERN_ERR "%s: IRQ but nothing to do!\n", __func__);
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (status & I2C_STATUS_ERROR_MASK) {
		err = -EIO;
		goto out_err;
	}

	if (dev->msg->flags & I2C_M_RD) {
		if (status & I2C_STATUS_RD_BUFFER_FULL) {

			/*
			 * Theres something in the FIFO.
			 * Are we expecting data or flush crap?
			 */
			if (dev->cnt) { /* DATA */
				uint8_t *data = &dev->msg->buf[dev->pos];

#if defined(CONFIG_KERNEL_MOTOROLA)
				*data = readl(dev->base + I2C_READ_DATA);
				dev->cnt--;
				dev->pos++;
				/* This is in spin-lock. So there will be no
				 * scheduling between reading the second-last
				 * byte and writing LAST_BYTE to the controller.
				 * So extra read-cycle-clock won't be generated
				 */
				if (dev->cnt == 1)
					writel(I2C_WRITE_DATA_LAST_BYTE,
						dev->base + I2C_WRITE_DATA);
				else if (dev->cnt == 0)
					goto out_complete;
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
				/* This is in spin-lock. So there will be no
				 * scheduling between reading the second-last
				 * byte and writing LAST_BYTE to the controller.
				 * So extra read-cycle-clock won't be generated
				 * Per I2C MSM HW Specs: Write LAST_BYTE befure
				 * reading 2nd last byte
				 */
				if (dev->cnt == 2)
					writel(I2C_WRITE_DATA_LAST_BYTE,
						dev->base + I2C_WRITE_DATA);
				*data = readl(dev->base + I2C_READ_DATA);
				dev->cnt--;
				dev->pos++;
				if (dev->msg->len == 1)
					dev->rd_acked = 0;
				if (dev->cnt == 0)
					goto out_complete;
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
			} else {
				/* Now that extra read-cycle-clocks aren't
				 * generated, this becomes error condition
				 */
				dev_err(dev->dev,
					"read did not stop, status - %x\n",
					status);
				err = -EIO;
				goto out_err;
			}
#if !defined(CONFIG_KERNEL_MOTOROLA)
		} else if (dev->msg->len == 1 && dev->rd_acked == 0 &&
				(status & I2C_STATUS_RX_DATA_STATE))
			writel(I2C_WRITE_DATA_LAST_BYTE,
				dev->base + I2C_WRITE_DATA);
#else /* !defined(CONFIG_KERNEL_MOTOROLA) */
		}
#endif /* !defined(CONFIG_KERNEL_MOTOROLA) */
	} else {
		uint16_t data;

		if (status & I2C_STATUS_WR_BUFFER_FULL) {
			dev_err(dev->dev,
				"Write buffer full in ISR on write?\n");
			err = -EIO;
			goto out_err;
		}

		if (dev->cnt) {
			/* Ready to take a byte */
			data = dev->msg->buf[dev->pos];
#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
			if (dev->cnt == 1)
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
			if (dev->cnt == 1 && dev->rem == 1)
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
				data |= I2C_WRITE_DATA_LAST_BYTE;

			writel(data, dev->base + I2C_WRITE_DATA);
			dev->pos++;
			dev->cnt--;
		} else
			goto out_complete;
	}

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;

 out_err:
	dev->err = err;
 out_complete:
	complete(dev->complete);
	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

static int
msm_i2c_poll_writeready(struct msm_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + I2C_STATUS);

		if (!(status & I2C_STATUS_WR_BUFFER_FULL))
			return 0;
		if (retries++ > 1000)
#if defined(CONFIG_MACH_MOT)
			msleep(1);
#else
			udelay(100);
#endif
	}
	return -ETIMEDOUT;
}

#if defined(CONFIG_KERNEL_MOTOROLA)
static int
msm_i2c_poll_notbusy(struct msm_i2c_dev *dev)
{
   uint32_t retries = 0;
   uint32_t triedToRecover=0;

   do {
      retries = 0;

      while (retries != 2000)
      {
         uint32_t status = readl(dev->base + I2C_STATUS);
         if (!(status & I2C_STATUS_BUS_ACTIVE))
            return 0;
         if (retries++ > 1000)
            udelay(10);
      }
      if (triedToRecover)
         return -ETIMEDOUT;

      gpio_tlmm_config(GPIO_CFG(dev->gpio_i2c_scl, 0, GPIO_OUTPUT,
         GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
      gpio_tlmm_config(GPIO_CFG(dev->gpio_i2c_sda, 0, GPIO_OUTPUT,
         GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
      udelay(50);

      gpio_configure(dev->gpio_i2c_scl, GPIOF_DRIVE_OUTPUT|GPIOF_OUTPUT_HIGH);
      gpio_configure(dev->gpio_i2c_sda, GPIOF_DRIVE_OUTPUT|GPIOF_OUTPUT_HIGH);
      udelay(50);

      gpio_tlmm_config(GPIO_CFG(dev->gpio_i2c_scl, 1, GPIO_OUTPUT,
         GPIO_PULL_UP, GPIO_16MA), GPIO_ENABLE);
      gpio_tlmm_config(GPIO_CFG(dev->gpio_i2c_sda, 1, GPIO_OUTPUT,
         GPIO_PULL_UP, GPIO_16MA), GPIO_ENABLE);
      udelay(50);
      triedToRecover = 1;
   } while(1);

   return -ETIMEDOUT;
}
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
static int
msm_i2c_poll_notbusy(struct msm_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + I2C_STATUS);

		if (!(status & I2C_STATUS_BUS_ACTIVE))
			return 0;
		if (retries++ > 1000)
#if defined(CONFIG_MACH_MOT)
			msleep(1);
#else
			udelay(100);
#endif
	}
	return -ETIMEDOUT;
}
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */

static void
msm_i2c_rmutex_lock(struct msm_i2c_dev *dev)
{
	int gotlock = 0;
	unsigned long flags;
	if (!dev->pdata->rmutex)
		return;
	do {
		remote_spin_lock_irqsave(&dev->rspin_lock, flags);
		if (*(dev->pdata->rmutex) == 0) {
			*(dev->pdata->rmutex) = 1;
			gotlock = 1;
		}
		remote_spin_unlock_irqrestore(&dev->rspin_lock, flags);
		/* wait for 1-byte clock interval */
		if (!gotlock)
			udelay(10000000/dev->pdata->clk_freq);
	} while (!gotlock);
}

static void
msm_i2c_rmutex_unlock(struct msm_i2c_dev *dev)
{
	unsigned long flags;
	if (!dev->pdata->rmutex)
		return;
	remote_spin_lock_irqsave(&dev->rspin_lock, flags);
	*(dev->pdata->rmutex) = 0;
	remote_spin_unlock_irqrestore(&dev->rspin_lock, flags);
}

static int
msm_i2c_recover_bus_busy(struct msm_i2c_dev *dev, struct i2c_adapter *adap)
{
	int i;
	int gpio_clk;
	int gpio_dat;
	uint32_t status = readl(dev->base + I2C_STATUS);
	bool gpio_clk_status = false;

	if (!(status & (I2C_STATUS_BUS_ACTIVE | I2C_STATUS_WR_BUFFER_FULL)))
		return 0;

	dev->pdata->msm_i2c_config_gpio(adap->nr, 0);
	/* Even adapter is primary and Odd adapter is AUX */
	if (adap->nr % 2) {
		gpio_clk = dev->pdata->aux_clk;
		gpio_dat = dev->pdata->aux_dat;
	} else {
		gpio_clk = dev->pdata->pri_clk;
		gpio_dat = dev->pdata->pri_dat;
	}
#ifndef CONFIG_MACH_MOT
	disable_irq(dev->irq);
#endif
	if (status & I2C_STATUS_RD_BUFFER_FULL) {
		dev_warn(dev->dev, "Read buffer full, status %x, intf %x\n",
			 status, readl(dev->base + I2C_INTERFACE_SELECT));
		writel(I2C_WRITE_DATA_LAST_BYTE, dev->base + I2C_WRITE_DATA);
		readl(dev->base + I2C_READ_DATA);
	} else if (status & I2C_STATUS_BUS_MASTER) {
		dev_warn(dev->dev, "Still the bus master, status %x, intf %x\n",
			 status, readl(dev->base + I2C_INTERFACE_SELECT));
		writel(I2C_WRITE_DATA_LAST_BYTE | 0xff,
		       dev->base + I2C_WRITE_DATA);
	}

	for (i = 0; i < 9; i++) {
		if (gpio_get_value(gpio_dat) && gpio_clk_status)
			break;
		gpio_direction_output(gpio_clk, 0);
		udelay(5);
		gpio_direction_output(gpio_dat, 0);
		udelay(5);
		gpio_direction_input(gpio_clk);
		udelay(5);
		if (!gpio_get_value(gpio_clk))
			udelay(20);
		if (!gpio_get_value(gpio_clk))
			msleep(10);
		gpio_clk_status = gpio_get_value(gpio_clk);
		gpio_direction_input(gpio_dat);
		udelay(5);
	}
	dev->pdata->msm_i2c_config_gpio(adap->nr, 1);
	udelay(10);

	status = readl(dev->base + I2C_STATUS);
	if (!(status & I2C_STATUS_BUS_ACTIVE)) {
		dev_info(dev->dev, "Bus busy cleared after %d clock cycles, "
			 "status %x, intf %x\n",
			 i, status, readl(dev->base + I2C_INTERFACE_SELECT));
#ifndef CONFIG_MACH_MOT
		enable_irq(dev->irq);
#endif
		return 0;
	}

	dev_err(dev->dev, "Bus still busy, status %x, intf %x\n",
		 status, readl(dev->base + I2C_INTERFACE_SELECT));
#ifndef CONFIG_MACH_MOT
	enable_irq(dev->irq);
#endif
	return -EBUSY;
}

static int
msm_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	DECLARE_COMPLETION_ONSTACK(complete);
	struct msm_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	int rem = num;
	uint16_t addr;
	long timeout;
	unsigned long flags;
	int check_busy = 1;

	mutex_lock(&dev->mlock);
	if (dev->suspended) {
		mutex_unlock(&dev->mlock);
		return -EIO;
	}

	/* Don't allow power collapse until we release remote spinlock */
	pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c",
					dev->pdata->pm_lat);
	msm_i2c_rmutex_lock(dev);
#ifndef CONFIG_MACH_MOT
	if (adap == &dev->adap_pri)
		writel(0, dev->base + I2C_INTERFACE_SELECT);
	else
		writel(I2C_INTERFACE_SELECT_INTF_SELECT,
				dev->base + I2C_INTERFACE_SELECT);
#endif
	enable_irq(dev->irq);
	while (rem) {
		addr = msgs->addr << 1;
		if (msgs->flags & I2C_M_RD)
			addr |= 1;

		spin_lock_irqsave(&dev->lock, flags);
		dev->msg = msgs;
#ifndef CONFIG_MACH_MOT
		dev->rem = rem;
#endif
		dev->pos = 0;
		dev->err = 0;
		dev->flush_cnt = 0;
		dev->cnt = msgs->len;
		dev->complete = &complete;
		spin_unlock_irqrestore(&dev->lock, flags);
		
#if defined(CONFIG_KERNEL_MOTOROLA)
		ret = msm_i2c_poll_notbusy(dev);
		if (ret) {
			dev_err(dev->dev, "Error waiting for notbusy\n");
			goto out_err;
		}
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
		if (check_busy) {
			ret = msm_i2c_poll_notbusy(dev);
			if (ret)
				ret = msm_i2c_recover_bus_busy(dev, adap);
				if (ret) {
					dev_err(dev->dev,
						"Error waiting for notbusy\n");
					goto out_err;
				}
			check_busy = 0;
		}
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */

		if (rem == 1 && msgs->len == 0)
			addr |= I2C_WRITE_DATA_LAST_BYTE;

		/* Wait for WR buffer not full */
		ret = msm_i2c_poll_writeready(dev);
		if (ret) {
#ifndef CONFIG_MACH_MOT
			ret = msm_i2c_recover_bus_busy(dev, adap);
			if (ret) {
				dev_err(dev->dev,
				"Error waiting for write ready before addr\n");
				goto out_err;
			}
#else
			dev_err(dev->dev,
				"Error waiting for write ready before addr\n");
			goto out_err;
#endif
		}

		/* special case for doing 1 byte read.
		 * There should be no scheduling between I2C controller becoming
		 * ready to read and writing LAST-BYTE to I2C controller
		 * This will avoid potential of I2C controller starting to latch
		 * another extra byte.
		 */
		if ((msgs->len == 1) && (msgs->flags & I2C_M_RD)) {
			uint32_t retries = 0;
			spin_lock_irqsave(&dev->lock, flags);

			writel(I2C_WRITE_DATA_ADDR_BYTE | addr,
				dev->base + I2C_WRITE_DATA);

			/* Poll for I2C controller going into RX_DATA mode to
			 * ensure controller goes into receive mode.
			 * Just checking write_buffer_full may not work since
			 * there is delay between the write-buffer becoming
			 * empty and the slave sending ACK to ensure I2C
			 * controller goes in receive mode to receive data.
			 */
			while (retries != 2000) {
				uint32_t status = readl(dev->base + I2C_STATUS);

					if (status & I2C_STATUS_RX_DATA_STATE)
						break;
				retries++;
			}
			if (retries >= 2000) {
#if defined(CONFIG_KERNEL_MOTOROLA)
				spin_unlock_irqrestore(&dev->lock, flags);
				dev_err(dev->dev,
					"Error doing one byte read @%x\n", addr);
		                ret = -EIO;
				goto out_err;
			}
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
				dev->rd_acked = 0;
				spin_unlock_irqrestore(&dev->lock, flags);
				/* 1-byte-reads from slow devices in interrupt
				 * context
				 */

				goto wait_for_int;
			}

			dev->rd_acked = 1;
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */

			writel(I2C_WRITE_DATA_LAST_BYTE,
					dev->base + I2C_WRITE_DATA);
			spin_unlock_irqrestore(&dev->lock, flags);
		} else {
			writel(I2C_WRITE_DATA_ADDR_BYTE | addr,
					 dev->base + I2C_WRITE_DATA);
		}
		/* Polling and waiting for write_buffer_empty is not necessary.
		 * Even worse, if we do, it can result in invalid status and
		 * error if interrupt(s) occur while polling.
		 */

		/*
		 * Now that we've setup the xfer, the ISR will transfer the data
		 * and wake us up with dev->err set if there was an error
		 */
wait_for_int:

		timeout = wait_for_completion_timeout(&complete, HZ);
		if (!timeout) {
#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
			dev_err(dev->dev, "Transaction timed out for addr 0x%x\n",
                msgs->addr);
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
			dev_err(dev->dev, "Transaction timed out\n");
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
			writel(I2C_WRITE_DATA_LAST_BYTE,
				dev->base + I2C_WRITE_DATA);
			msleep(100);
			/* FLUSH */
			readl(dev->base + I2C_READ_DATA);
			readl(dev->base + I2C_STATUS);
			ret = -ETIMEDOUT;
			goto out_err;
		}
		if (dev->err) {
#ifdef CONFIG_MACH_MOT
            if (! (dev->msg->flags & I2C_M_NAK_LIKELY))
#endif
			dev_err(dev->dev,
#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
				"Error during data xfer (%d) at addr 0x%x\n",
				dev->err, msgs->addr);
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
				"Error during data xfer (%d)\n",
				dev->err);
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
			ret = dev->err;
#ifdef CONFIG_MACH_MOT
			msm_i2c_recover_bus_busy(dev, adap);
#endif
			goto out_err;
		}

		if (msgs->flags & I2C_M_RD)
			check_busy = 1;

		msgs++;
		rem--;
	}

	ret = num;
 out_err:
	spin_lock_irqsave(&dev->lock, flags);
	dev->complete = NULL;
	dev->msg = NULL;
#ifndef CONFIG_MACH_MOT
	dev->rem = 0;
#endif
	dev->pos = 0;
	dev->err = 0;
	dev->flush_cnt = 0;
	dev->cnt = 0;
	spin_unlock_irqrestore(&dev->lock, flags);
	disable_irq(dev->irq);
	msm_i2c_rmutex_unlock(dev);
	pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c",
					PM_QOS_DEFAULT_VALUE);
	mutex_unlock(&dev->mlock);
	return ret;
}

static u32
msm_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm msm_i2c_algo = {
	.master_xfer	= msm_i2c_xfer,
	.functionality	= msm_i2c_func,
};

static int
msm_i2c_probe(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev;
	struct resource		*mem, *irq, *ioarea;
	int ret;
	int fs_div;
	int hs_div;
	int i2c_clk;
	int clk_ctl;
	struct clk *clk;
	struct msm_i2c_platform_data *pdata;

	printk(KERN_INFO "msm_i2c_probe\n");

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}
	clk = clk_get(&pdev->dev, "i2c_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Could not get clock\n");
		ret = PTR_ERR(clk);
		goto err_clk_get_failed;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform data not initialized\n");
		ret = -ENOSYS;
		goto err_clk_get_failed;
	}
	if (!pdata->msm_i2c_config_gpio) {
		dev_err(&pdev->dev, "config_gpio function not initialized\n");
		ret = -ENOSYS;
		goto err_clk_get_failed;
	}
	/* We support frequencies upto FAST Mode(400KHz) */
	if (pdata->clk_freq <= 0 || pdata->clk_freq > 400000) {
		dev_err(&pdev->dev, "clock frequency not supported\n");
		ret = -EIO;
		goto err_clk_get_failed;
	}

	dev = kzalloc(sizeof(struct msm_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_alloc_dev_failed;
	}

	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	dev->clk = clk;
	dev->pdata = pdata;
#ifdef CONFIG_KERNEL_MOTOROLA
	dev->gpio_i2c_scl = pdata->gpio_i2c_scl;
	dev->gpio_i2c_sda = pdata->gpio_i2c_sda;
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
	dev->base = ioremap(mem->start, (mem->end - mem->start) + 1);
	if (!dev->base) {
		ret = -ENOMEM;
		goto err_ioremap_failed;
	}

	spin_lock_init(&dev->lock);
	platform_set_drvdata(pdev, dev);

	clk_enable(clk);

	if (pdata->rmutex != NULL)
		remote_spin_lock_init(&dev->rspin_lock, pdata->rsl_id);
	/* I2C_HS_CLK = I2C_CLK/(3*(HS_DIVIDER_VALUE+1) */
	/* I2C_FS_CLK = I2C_CLK/(2*(FS_DIVIDER_VALUE+3) */
	/* FS_DIVIDER_VALUE = ((I2C_CLK / I2C_FS_CLK) / 2) - 3 */
	i2c_clk = 19200000; /* input clock */
	fs_div = ((i2c_clk / pdata->clk_freq) / 2) - 3;
#ifdef CONFIG_MACH_MOT
	/* 
		Divider formula from the spec gives wrong clock.
		Experimentally it acts like i2c_clk = tcxo/(2*(fs_div+6))
		Reduce fs_div by 3 to compensate
	*/
	fs_div -= 3; 
#endif
	hs_div = 3;
	clk_ctl = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
	writel(clk_ctl, dev->base + I2C_CLK_CTL);
#ifdef CONFIG_MACH_MOT
	printk(KERN_INFO "msm_i2c_probe: clk_ctl %x, %d Hz\n",
	       clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 6)));
#else
	printk(KERN_INFO "msm_i2c_probe: clk_ctl %x, %d Hz\n",
	       clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 3)));
#endif

#ifndef CONFIG_MACH_MOT

	i2c_set_adapdata(&dev->adap_pri, dev);
	dev->adap_pri.algo = &msm_i2c_algo;
	strlcpy(dev->adap_pri.name,
		"MSM I2C adapter-PRI",
		sizeof(dev->adap_pri.name));

	dev->adap_pri.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adap_pri);
	if (ret) {
		dev_err(&pdev->dev, "Primary i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}

	i2c_set_adapdata(&dev->adap_aux, dev);
	dev->adap_aux.algo = &msm_i2c_algo;
	strlcpy(dev->adap_aux.name,
		"MSM I2C adapter-AUX",
		sizeof(dev->adap_aux.name));

	dev->adap_aux.nr = pdev->id + 1;
	ret = i2c_add_numbered_adapter(&dev->adap_aux);
	if (ret) {
		dev_err(&pdev->dev, "auxiliary i2c_add_adapter failed\n");
		i2c_del_adapter(&dev->adap_pri);
		goto err_i2c_add_adapter_failed;
	}
#else
	i2c_set_adapdata(&dev->adapter, dev);
	dev->adapter.algo = &msm_i2c_algo;
	strncpy(dev->adapter.name,
		"MSM I2C adapter",
		sizeof(dev->adapter.name));

	dev->adapter.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}
#endif
	ret = request_irq(dev->irq, msm_i2c_interrupt,
			IRQF_TRIGGER_RISING, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c",
					PM_QOS_DEFAULT_VALUE);
	disable_irq(dev->irq);
	dev->suspended = 0;
	mutex_init(&dev->mlock);
	/* Config GPIOs for primary and secondary lines */
#ifndef CONFIG_MACH_MOT
	pdata->msm_i2c_config_gpio(dev->adap_pri.nr, 1);
	pdata->msm_i2c_config_gpio(dev->adap_aux.nr, 1);
#else
	pdata->msm_i2c_config_gpio(dev->adapter.nr, 1);
	pdata->msm_i2c_config_gpio(dev->adapter.nr + 1, 1);
#endif

	return 0;

/*	free_irq(dev->irq, dev); */
err_request_irq_failed:
#ifndef CONFIG_MACH_MOT
	i2c_del_adapter(&dev->adap_pri);
	i2c_del_adapter(&dev->adap_aux);
#else
	i2c_del_adapter(&dev->adapter);
#endif
err_i2c_add_adapter_failed:
	clk_disable(clk);
	iounmap(dev->base);
err_ioremap_failed:
	kfree(dev);
err_alloc_dev_failed:
	clk_put(clk);
err_clk_get_failed:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int
msm_i2c_remove(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	platform_set_drvdata(pdev, NULL);
	pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c");
	free_irq(dev->irq, dev);
#ifndef CONFIG_MACH_MOT
	i2c_del_adapter(&dev->adap_pri);
	i2c_del_adapter(&dev->adap_aux);
#else
	i2c_del_adapter(&dev->adapter);
#endif
	clk_disable(dev->clk);
	clk_put(dev->clk);
	iounmap(dev->base);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

static int msm_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct msm_i2c_dev *dev = platform_get_drvdata(pdev);
	/* Wait until current transaction finishes
	 * Make sure remote lock is released before we suspend
	 */
	if (dev) {
		mutex_lock(&dev->mlock);
		dev->suspended = 1;
		mutex_unlock(&dev->mlock);
		clk_disable(dev->clk);
	}

	return 0;
}

static int msm_i2c_resume(struct platform_device *pdev)
{
	struct msm_i2c_dev *dev = platform_get_drvdata(pdev);
	if (dev) {
		clk_enable(dev->clk);
		mutex_lock(&dev->mlock);		
		dev->suspended = 0;
		mutex_unlock(&dev->mlock);
	}
	return 0;
}

static struct platform_driver msm_i2c_driver = {
	.probe		= msm_i2c_probe,
	.remove		= msm_i2c_remove,
	.suspend	= msm_i2c_suspend,
	.resume		= msm_i2c_resume,
	.driver		= {
		.name	= "msm_i2c",
		.owner	= THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
msm_i2c_init_driver(void)
{
	return platform_driver_register(&msm_i2c_driver);
}
subsys_initcall(msm_i2c_init_driver);

static void __exit msm_i2c_exit_driver(void)
{
	platform_driver_unregister(&msm_i2c_driver);
}
module_exit(msm_i2c_exit_driver);

