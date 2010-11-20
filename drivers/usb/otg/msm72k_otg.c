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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>

#include <linux/device.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm72k_otg.h>
#include <mach/msm_hsusb.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
<<<<<<< HEAD:drivers/usb/otg/msm72k_otg.c
#include <linux/pm_qos_params.h>
=======
>>>>>>> QCOMM_4715_cs:drivers/usb/otg/msm72k_otg.c

#define MSM_USB_BASE	(dev->regs)
#define is_host()	((OTGSC_ID & readl(USB_OTGSC)) ? 0 : 1)
#define is_b_sess_vld()	((OTGSC_BSV & readl(USB_OTGSC)) ? 1 : 0)
#define DRIVER_NAME	"msm_otg"

static void otg_reset(struct msm_otg *dev);

struct msm_otg *the_msm_otg;

static unsigned ulpi_read(struct msm_otg *dev, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct msm_otg *dev, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		;

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

static void enable_idgnd(struct msm_otg *dev)
{
	ulpi_write(dev, (1<<4), 0x0E);
	ulpi_write(dev, (1<<4), 0x11);
	writel(readl(USB_OTGSC) | OTGSC_IDIE, USB_OTGSC);
}

static void disable_idgnd(struct msm_otg *dev)
{
	ulpi_write(dev, (1<<4), 0x0F);
	ulpi_write(dev, (1<<4), 0x12);
	writel(readl(USB_OTGSC) & ~OTGSC_IDIE, USB_OTGSC);
}

static void enable_sess_valid(struct msm_otg *dev)
{
	ulpi_write(dev, (1<<2), 0x0E);
	ulpi_write(dev, (1<<2), 0x11);
	writel(readl(USB_OTGSC) | OTGSC_BSVIE, USB_OTGSC);
}

static void disable_sess_valid(struct msm_otg *dev)
{
	ulpi_write(dev, (1<<2), 0x0F);
	ulpi_write(dev, (1<<2), 0x12);
	writel(readl(USB_OTGSC) & ~OTGSC_BSVIE, USB_OTGSC);
}

int release_wlocks;
struct dentry *debugfs_dent;
struct dentry *rel_wlocks_file;
#if defined(CONFIG_DEBUG_FS)
static ssize_t debug_read_release_wlocks(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	char kbuf[100];
	size_t c = 0;

	memset(kbuf, 0, 100);

	c = scnprintf(kbuf, 100, "%d", release_wlocks);

	if (copy_to_user(ubuf, kbuf, c))
		return -EFAULT;

	return c;
}
static ssize_t debug_write_release_wlocks(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char kbuf[100];
	long temp;

	memset(kbuf, 0, 100);

	if (copy_from_user(kbuf, buf, count > 99 ? 99 : count))
		return -EFAULT;

	if (strict_strtol(kbuf, 10, &temp))
		return -EINVAL;

	if (temp)
		release_wlocks = 1;
	else
		release_wlocks = 0;

	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_wlocks_ops = {
	.open = debug_open,
	.read = debug_read_release_wlocks,
	.write = debug_write_release_wlocks,
};

static void msm_otg_debugfs_init(struct msm_otg *dev)
{
	debugfs_dent = debugfs_create_dir("otg", 0);

	if (IS_ERR(debugfs_dent) || !debugfs_dent)
		return;

	rel_wlocks_file = debugfs_create_file("release_wlocks", 0666,
				debugfs_dent, dev, &debug_wlocks_ops);

	return;
}

static void msm_otg_debugfs_cleanup(void)
{
	if (rel_wlocks_file && !IS_ERR(rel_wlocks_file))
		debugfs_remove(rel_wlocks_file);

	if (debugfs_dent && !IS_ERR(debugfs_dent))
		debugfs_remove(debugfs_dent);
}

#else

static void msm_otg_debugfs_init(struct msm_otg *dev) { }
static void msm_otg_debugfs_cleanup() { }

#endif
<<<<<<< HEAD:drivers/usb/otg/msm72k_otg.c
static void msm_otg_suspend_locks_acquire(struct msm_otg *dev, int acquire)
{
	if (acquire) {
		wake_lock(&dev->wlock);
		pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
				DRIVER_NAME, 0);
	} else {
		wake_unlock(&dev->wlock);
		pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
				DRIVER_NAME, PM_QOS_DEFAULT_VALUE);
	}
}

static void msm_otg_suspend_locks_init(struct msm_otg *dev, int init)
{
	if (init) {
		wake_lock_init(&dev->wlock,
				WAKE_LOCK_SUSPEND, "usb_bus_active");
		pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY, DRIVER_NAME,
							PM_QOS_DEFAULT_VALUE);
		msm_otg_suspend_locks_acquire(dev, 1);
	} else {
		wake_lock_destroy(&dev->wlock);
		pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY, DRIVER_NAME);
	}
}
=======
>>>>>>> QCOMM_4715_cs:drivers/usb/otg/msm72k_otg.c

static void msm_otg_start_peripheral(struct otg_transceiver *xceiv, int on)
{
	if (!xceiv->gadget)
		return;

	if (on)
		usb_gadget_vbus_connect(xceiv->gadget);
	else
		usb_gadget_vbus_disconnect(xceiv->gadget);
}

static void msm_otg_start_host(struct otg_transceiver *xceiv, int on)
{
	if (!xceiv->host)
		return;

	/* TBD: call host specific start function */
}

static int msm_otg_suspend(struct msm_otg *dev)
{
	unsigned long timeout;
	int vbus = 0;

	disable_irq(dev->irq);
	if (dev->in_lpm)
		goto out;

	otg_reset(dev);

	ulpi_read(dev, 0x14);/* clear PHY interrupt latch register */
	ulpi_write(dev, 0x01, 0x30);/* PHY comparators on in LPM */
	ulpi_write(dev, 0x08, 0x09);/* turn off PLL on integrated phy */

	timeout = jiffies + msecs_to_jiffies(500);
	disable_phy_clk();
	while (!is_phy_clk_disabled()) {
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Unable to suspend phy\n", __func__);
			otg_reset(dev);
			goto out;
		}
		msleep(1);
	}

	writel(readl(USB_USBCMD) | ASYNC_INTR_CTRL | ULPI_STP_CTRL, USB_USBCMD);
	clk_disable(dev->clk);
	clk_disable(dev->pclk);
	if (device_may_wakeup(dev->otg.dev))
		enable_irq_wake(dev->irq);
	dev->in_lpm = 1;

	/* TBD: as there is no bus suspend implemented as of now
	 * it should be dummy check
	 */
	if (!vbus || release_wlocks)
<<<<<<< HEAD:drivers/usb/otg/msm72k_otg.c
		msm_otg_suspend_locks_acquire(dev, 0);
=======
		wake_unlock(&dev->wlock);
>>>>>>> QCOMM_4715_cs:drivers/usb/otg/msm72k_otg.c

	pr_info("%s: usb in low power mode\n", __func__);
out:
	enable_irq(dev->irq);

	return 0;
}

static int msm_otg_resume(struct msm_otg *dev)
{
	unsigned temp;

	if (!dev->in_lpm)
		return 0;

<<<<<<< HEAD:drivers/usb/otg/msm72k_otg.c
	msm_otg_suspend_locks_acquire(dev, 1);
=======
	wake_lock(&dev->wlock);
>>>>>>> QCOMM_4715_cs:drivers/usb/otg/msm72k_otg.c

	clk_enable(dev->clk);
	clk_enable(dev->pclk);

	temp = readl(USB_USBCMD);
	temp &= ~ASYNC_INTR_CTRL;
	temp &= ~ULPI_STP_CTRL;
	writel(temp, USB_USBCMD);

	if (device_may_wakeup(dev->otg.dev))
		disable_irq_wake(dev->irq);

	dev->in_lpm = 0;
	pr_info("%s: usb exited from low power mode\n", __func__);

	return 0;
}

static int msm_otg_set_suspend(struct otg_transceiver *xceiv, int suspend)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	if (suspend)
		msm_otg_suspend(dev);
	else {
		unsigned long timeout;

		disable_irq(dev->irq);

		msm_otg_resume(dev);

		if (!is_phy_clk_disabled())
			goto out;

		timeout = jiffies + msecs_to_jiffies(500);
		enable_phy_clk();
		while (is_phy_clk_disabled()) {
			if (time_after(jiffies, timeout)) {
				pr_err("%s: Unable to wakeup phy\n", __func__);
				otg_reset(dev);
				break;
			}
			msleep(1);
		}
out:
		enable_irq(dev->irq);
	}

	return 0;
}

static int msm_otg_set_peripheral(struct otg_transceiver *xceiv,
			struct usb_gadget *gadget)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	if (!gadget) {
		msm_otg_start_peripheral(xceiv, 0);
		dev->otg.gadget = 0;
		disable_sess_valid(dev);
		return 0;
	}
	dev->otg.gadget = gadget;
	enable_sess_valid(dev);
	pr_info("peripheral driver registered w/ tranceiver\n");

	if (is_b_sess_vld())
		msm_otg_start_peripheral(&dev->otg, 1);
	else if (is_host())
		msm_otg_start_host(&dev->otg, 1);
	else
		msm_otg_suspend(dev);

	return 0;
}

static int msm_otg_set_host(struct otg_transceiver *xceiv, struct usb_bus *host)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	if (!host) {
		msm_otg_start_host(xceiv, 0);
		dev->otg.host = 0;
		disable_idgnd(dev);
		return 0;
	}
	dev->otg.host = host;
	enable_idgnd(dev);
	pr_info("host driver registered w/ tranceiver\n");

#ifndef CONFIG_USB_GADGET_MSM_72K
	if (is_host())
		msm_otg_start_host(&dev->otg, 1);
	else
		msm_otg_suspend(dev);
#endif
	return 0;
}

static irqreturn_t msm_otg_irq(int irq, void *data)
{
	struct msm_otg *dev = data;
	u32 otgsc = 0;

	if (dev->in_lpm) {
		msm_otg_resume(dev);
		return IRQ_HANDLED;
	}

	otgsc = readl(USB_OTGSC);
	if (!otgsc & OTGSC_INTR_STS_MASK)
		return IRQ_HANDLED;

	if ((otgsc & OTGSC_IDIS) && (otgsc & OTGSC_IDIE)) {
		pr_info("ID -> (%s)\n", (otgsc & OTGSC_ID) ? "B" : "A");
		msm_otg_start_host(&dev->otg, is_host());
	} else if ((otgsc & OTGSC_BSVIS) && (otgsc & OTGSC_BSVIE)) {
		pr_info("VBUS - (%s)\n", otgsc & OTGSC_BSV ? "ON" : "OFF");
		if (!is_host())
			msm_otg_start_peripheral(&dev->otg, is_b_sess_vld());
	}
	writel(otgsc, USB_OTGSC);

	return IRQ_HANDLED;
}

#define USB_LINK_RESET_TIMEOUT	(msecs_to_jiffies(10))
static void otg_reset(struct msm_otg *dev)
{
	unsigned long timeout;

	if (dev->phy_reset)
		dev->phy_reset();
	/*disable all phy interrupts*/
	ulpi_write(dev, 0x1F, 0x0F);
	ulpi_write(dev, 0x1F, 0x12);
	msleep(100);

	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	while (readl(USB_USBCMD) & USBCMD_RESET) {
		if (time_after(jiffies, timeout)) {
			pr_err("msm_otg: usb link reset timeout\n");
			break;
		}
		msleep(1);
	}

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	ulpi_write(dev, ULPI_AMPLITUDE, ULPI_CONFIG_REG);
	writel(0x0, USB_AHB_BURST);
	writel(0x00, USB_AHB_MODE);

	if (dev->otg.gadget)
		enable_sess_valid(dev);
	if (dev->otg.host)
		enable_idgnd(dev);
}

static int __init msm_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct msm_otg *dev;
	struct msm_otg_platform_data *pdata;

	dev = kzalloc(sizeof(struct msm_otg), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->otg.dev = &pdev->dev;

	if (pdev->dev.platform_data) {
		pdata = pdev->dev.platform_data;
		dev->rpc_connect = pdata->rpc_connect;
		dev->phy_reset = pdata->phy_reset;
	}

	if (dev->rpc_connect) {
		ret = dev->rpc_connect(1);
		pr_info("%s: rpc_connect(%d)\n", __func__, ret);
		if (ret) {
			pr_err("%s: rpc connect failed\n", __func__);
			ret = -ENODEV;
			goto free_dev;
		}
	}

	dev->clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(dev->clk)) {
		pr_err("%s: failed to get usb_hs_clk\n", __func__);
		ret = PTR_ERR(dev->clk);
		goto rpc_fail;
	}
	dev->pclk = clk_get(&pdev->dev, "usb_hs_pclk");
	if (IS_ERR(dev->clk)) {
		pr_err("%s: failed to get usb_hs_pclk\n", __func__);
		ret = PTR_ERR(dev->pclk);
		goto put_clk;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("%s: failed to get platform resource mem\n", __func__);
		ret = -ENODEV;
		goto put_pclk;
	}

	dev->regs = ioremap(res->start, resource_size(res));
	if (!dev->regs) {
		pr_err("%s: ioremap failed\n", __func__);
		ret = -ENOMEM;
		goto put_pclk;
	}
	dev->irq = platform_get_irq(pdev, 0);
	if (!dev->irq) {
		pr_err("%s: platform_get_irq failed\n", __func__);
		ret = -ENODEV;
		goto free_regs;
	}

	/* enable clocks */
	clk_enable(dev->clk);
	clk_enable(dev->pclk);

	otg_reset(dev);

	ret = request_irq(dev->irq, msm_otg_irq, IRQF_SHARED,
					"msm_otg", dev);
	if (ret) {
		pr_info("%s: request irq failed\n", __func__);
		clk_disable(dev->clk);
		clk_disable(dev->pclk);
		goto free_regs;
	}

	the_msm_otg = dev;
	dev->otg.set_peripheral = msm_otg_set_peripheral;
	dev->otg.set_host = msm_otg_set_host;
	dev->otg.set_suspend = msm_otg_set_suspend;
	if (otg_set_transceiver(&dev->otg)) {
		WARN_ON(1);
		goto free_regs;
	}

<<<<<<< HEAD:drivers/usb/otg/msm72k_otg.c
	msm_otg_suspend_locks_init(dev, 1);
=======
	wake_lock_init(&dev->wlock,
			WAKE_LOCK_SUSPEND, "usb_bus_active");
	wake_lock(&dev->wlock);
>>>>>>> QCOMM_4715_cs:drivers/usb/otg/msm72k_otg.c
	msm_otg_debugfs_init(dev);
	device_init_wakeup(&pdev->dev, 1);

	return 0;
free_regs:
	iounmap(dev->regs);
put_pclk:
	clk_put(dev->pclk);
put_clk:
	clk_put(dev->clk);
rpc_fail:
	dev->rpc_connect(0);
free_dev:
	kfree(dev);
	return ret;
}

static int __exit msm_otg_remove(struct platform_device *pdev)
{
	struct msm_otg *dev = the_msm_otg;

	free_irq(dev->irq, pdev);
	iounmap(dev->regs);
	clk_disable(dev->pclk);
	clk_disable(dev->clk);
	clk_put(dev->pclk);
	clk_put(dev->clk);
<<<<<<< HEAD:drivers/usb/otg/msm72k_otg.c
	msm_otg_suspend_locks_init(dev, 0);
=======
	wake_lock_destroy(&dev->wlock);
>>>>>>> QCOMM_4715_cs:drivers/usb/otg/msm72k_otg.c
	msm_otg_debugfs_cleanup();
	kfree(dev);
	if (dev->rpc_connect)
		dev->rpc_connect(0);
	return 0;
}

static struct platform_driver msm_otg_driver = {
	.remove = __exit_p(msm_otg_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_otg_init(void)
{
	return platform_driver_probe(&msm_otg_driver, msm_otg_probe);
}

static void __exit msm_otg_exit(void)
{
	platform_driver_unregister(&msm_otg_driver);
}

module_init(msm_otg_init);
module_exit(msm_otg_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("MSM usb transceiver driver");
MODULE_VERSION("1.00");
