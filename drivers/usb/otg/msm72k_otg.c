/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
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
#include <mach/clk.h>

#define MSM_USB_BASE	(dev->regs)
#define USB_LINK_RESET_TIMEOUT	(msecs_to_jiffies(10))
#define DRIVER_NAME	"msm_otg"

static void otg_reset(struct otg_transceiver *xceiv, int phy_reset);
static void msm_otg_set_vbus_state(int online);

struct msm_otg *the_msm_otg;

static int is_host(void)
{
	struct msm_otg *dev = the_msm_otg;

	if (dev->pdata->otg_mode == OTG_ID)
		return (OTGSC_ID & readl(USB_OTGSC)) ? 0 : 1;
	else
		return (dev->otg.state >= OTG_STATE_A_IDLE);
}

static int is_b_sess_vld(void)
{
	struct msm_otg *dev = the_msm_otg;

	if (dev->pdata->otg_mode == OTG_ID)
		return (OTGSC_BSV & readl(USB_OTGSC)) ? 1 : 0;
	else
		return (dev->otg.state == OTG_STATE_B_PERIPHERAL);
}

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

#ifdef CONFIG_USB_EHCI_MSM
static void enable_idgnd(struct msm_otg *dev)
{
	/* Do nothing if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return;

	ulpi_write(dev, (1<<4), 0x0E);
	ulpi_write(dev, (1<<4), 0x11);
	writel(readl(USB_OTGSC) | OTGSC_IDIE, USB_OTGSC);
}

static void disable_idgnd(struct msm_otg *dev)
{
	/* Do nothing if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return;

	ulpi_write(dev, (1<<4), 0x0F);
	ulpi_write(dev, (1<<4), 0x12);
	writel(readl(USB_OTGSC) & ~OTGSC_IDIE, USB_OTGSC);
}
#endif

static void enable_sess_valid(struct msm_otg *dev)
{
	/* Do nothing if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return;

	ulpi_write(dev, (1<<2), 0x0E);
	ulpi_write(dev, (1<<2), 0x11);
	writel(readl(USB_OTGSC) | OTGSC_BSVIE, USB_OTGSC);
}

static void disable_sess_valid(struct msm_otg *dev)
{
	/* Do nothing if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return;

	ulpi_write(dev, (1<<2), 0x0F);
	ulpi_write(dev, (1<<2), 0x12);
	writel(readl(USB_OTGSC) & ~OTGSC_BSVIE, USB_OTGSC);
}

static inline void set_pre_emphasis_level(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->pemp_level == PRE_EMPHASIS_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_CONFIG_REG3);
	res &= ~(ULPI_PRE_EMPHASIS_MASK);
	if (dev->pdata->pemp_level != PRE_EMPHASIS_DISABLE)
		res |= dev->pdata->pemp_level;
	ulpi_write(dev, res, ULPI_CONFIG_REG3);
}

static inline void set_cdr_auto_reset(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->cdr_autoreset == CDR_AUTO_RESET_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_DIGOUT_CTRL);
	if (dev->pdata->cdr_autoreset == CDR_AUTO_RESET_ENABLE)
		res &=  ~ULPI_CDR_AUTORESET;
	else
		res |=  ULPI_CDR_AUTORESET;
	ulpi_write(dev, res, ULPI_DIGOUT_CTRL);
}

static inline void set_driver_amplitude(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->drv_ampl == HS_DRV_AMPLITUDE_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_CONFIG_REG2);
	res &= ~ULPI_DRV_AMPL_MASK;
	if (dev->pdata->drv_ampl != HS_DRV_AMPLITUDE_ZERO_PERCENT)
		res |= dev->pdata->drv_ampl;
	ulpi_write(dev, res, ULPI_CONFIG_REG2);
}

static const char *state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:	return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:	return "a_wait_bcon";
	case OTG_STATE_A_HOST:		return "a_host";
	case OTG_STATE_A_SUSPEND:	return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:	return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:	return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:	return "a_vbus_err";
	case OTG_STATE_B_IDLE:		return "b_idle";
	case OTG_STATE_B_SRP_INIT:	return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:	return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:	return "b_wait_acon";
	case OTG_STATE_B_HOST:		return "b_host";
	default:			return "UNDEFINED";
	}
}

static const char *timer_string(int bit)
{
	switch (bit) {
	case A_WAIT_VRISE:		return "a_wait_vrise";
	case A_WAIT_VFALL:		return "a_wait_vfall";
	case B_SRP_FAIL:		return "b_srp_fail";
	case A_WAIT_BCON:		return "a_wait_bcon";
	case A_AIDL_BDIS:		return "a_aidl_bdis";
	case A_BIDL_ADIS:		return "a_bidl_adis";
	case B_ASE0_BRST:		return "b_ase0_brst";
	default:			return "UNDEFINED";
	}
}

/* Controller gives interrupt for every 1 mesc if 1MSIE is set in OTGSC.
 * This interrupt can be used as a timer source and OTG timers can be
 * implemented. But hrtimers on MSM hardware can give atleast 1/32 KHZ
 * precision. This precision is more than enough for OTG timers.
 */
static enum hrtimer_restart msm_otg_timer_func(struct hrtimer *_timer)
{
	struct msm_otg *dev = container_of(_timer, struct msm_otg, timer);

	/* Phy lockup issues are observed when VBUS Valid interrupt is
	 * enabled. Hence set A_VBUS_VLD upon timer exipration.
	 */
	if (dev->active_tmout == A_WAIT_VRISE)
		set_bit(A_VBUS_VLD, &dev->inputs);
	else
		set_bit(dev->active_tmout, &dev->tmouts);

	pr_debug("expired %s timer\n", timer_string(dev->active_tmout));
	queue_work(dev->wq, &dev->sm_work);
	return HRTIMER_NORESTART;
}

static void msm_otg_del_timer(struct msm_otg *dev)
{
	int bit = dev->active_tmout;

	pr_debug("deleting %s timer. remaining %lld msec \n", timer_string(bit),
			div_s64(ktime_to_us(hrtimer_get_remaining(&dev->timer)),
					1000));
	hrtimer_cancel(&dev->timer);
	clear_bit(bit, &dev->tmouts);
}

static void msm_otg_start_timer(struct msm_otg *dev, int time, int bit)
{
	clear_bit(bit, &dev->tmouts);
	dev->active_tmout = bit;
	pr_debug("starting %s timer\n", timer_string(bit));
	hrtimer_start(&dev->timer,
			ktime_set(time / 1000, (time % 1000) * 1000000),
			HRTIMER_MODE_REL);
}

/* No two otg timers run in parallel. So one hrtimer is sufficient */
static void msm_otg_init_timer(struct msm_otg *dev)
{
	hrtimer_init(&dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->timer.function = msm_otg_timer_func;
}

static const char *event_string(enum usb_otg_event event)
{
	switch (event) {
	case OTG_EVENT_DEV_CONN_TMOUT:
		return "DEV_CONN_TMOUT";
	case OTG_EVENT_NO_RESP_FOR_HNP_ENABLE:
		return "NO_RESP_FOR_HNP_ENABLE";
	case OTG_EVENT_HUB_NOT_SUPPORTED:
		return "HUB_NOT_SUPPORTED";
	case OTG_EVENT_DEV_NOT_SUPPORTED:
		return "DEV_NOT_SUPPORTED,";
	case OTG_EVENT_HNP_FAILED:
		return "HNP_FAILED";
	case OTG_EVENT_NO_RESP_FOR_SRP:
		return "NO_RESP_FOR_SRP";
	default:
		return "UNDEFINED";
	}
}

static int msm_otg_send_event(struct otg_transceiver *xceiv,
				enum usb_otg_event event)
{
	char module_name[16];
	char udev_event[128];
	char *envp[] = { module_name, udev_event, NULL };
	int ret;

	pr_debug("sending %s event\n", event_string(event));

	snprintf(module_name, 16, "MODULE=%s", DRIVER_NAME);
	snprintf(udev_event, 128, "EVENT=%s", event_string(event));
	ret = kobject_uevent_env(&xceiv->dev->kobj, KOBJ_CHANGE, envp);
	if (ret < 0)
		pr_info("uevent sending failed with ret = %d\n", ret);
	return ret;
}

static int msm_otg_start_hnp(struct otg_transceiver *xceiv)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	enum usb_otg_state state;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	state = dev->otg.state;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (state != OTG_STATE_A_HOST) {
		pr_err("HNP can not be initiated in %s state\n",
				state_string(state));
		return -EINVAL;
	}

	pr_debug("A-Host: HNP initiated\n");
	clear_bit(A_BUS_REQ, &dev->inputs);
	wake_lock(&dev->wlock);
	queue_work(dev->wq, &dev->sm_work);
	return 0;
}

static int msm_otg_start_srp(struct otg_transceiver *xceiv)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	u32	val;
	int ret = 0;
	enum usb_otg_state state;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	state = dev->otg.state;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (state != OTG_STATE_B_IDLE) {
		pr_err("SRP can not be initiated in %s state\n",
				state_string(state));
		ret = -EINVAL;
		goto out;
	}

	if ((jiffies - dev->b_last_se0_sess) < msecs_to_jiffies(TB_SRP_INIT)) {
		pr_debug("initial conditions of SRP are not met. Try again"
				"after some time\n");
		ret = -EAGAIN;
		goto out;
	}

	/* Harware auto assist data pulsing: Data pulse is given
	 * for 7msec; wait for vbus
	 */
	val = readl(USB_OTGSC);
	writel((val & ~OTGSC_INTR_STS_MASK) | OTGSC_HADP, USB_OTGSC);

	/* VBUS plusing is obsoleted in OTG 2.0 supplement */
out:
	return ret;
}

static int msm_otg_set_clk(struct otg_transceiver *xceiv, int on)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	if (on)
		/* enable clocks */
		clk_enable(dev->hs_clk);
	else
		clk_disable(dev->hs_clk);

	return 0;
}
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
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = dev->pdata;

	if (!xceiv->host)
		return;

	if (dev->start_host) {
		/* Some targets, e.g. ST1.5, use GPIO to choose b/w connector */
		if (on && pdata->setup_gpio)
			pdata->setup_gpio(on);
		dev->start_host(xceiv->host, on);
		if (!on && pdata->setup_gpio)
			pdata->setup_gpio(on);
	}
}

static int msm_otg_suspend(struct msm_otg *dev)
{
	unsigned long timeout;
	int vbus = 0;

	disable_irq(dev->irq);
	if (atomic_read(&dev->in_lpm))
		goto out;

	ulpi_read(dev, 0x14);/* clear PHY interrupt latch register */
	/* If there is no pmic notify support turn on phy comparators. */
	if (!dev->pmic_notif_supp)
		ulpi_write(dev, 0x01, 0x30);
	ulpi_write(dev, 0x08, 0x09);/* turn off PLL on integrated phy */

	timeout = jiffies + msecs_to_jiffies(500);
	disable_phy_clk();
	while (!is_phy_clk_disabled()) {
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Unable to suspend phy\n", __func__);
			/* Reset both phy and link */
			otg_reset(&dev->otg, 1);
			goto out;
		}
		msleep(1);
	}

	writel(readl(USB_USBCMD) | ASYNC_INTR_CTRL | ULPI_STP_CTRL, USB_USBCMD);
	if (dev->hs_pclk)
		clk_disable(dev->hs_pclk);
	if (dev->hs_cclk)
		clk_disable(dev->hs_cclk);
	if (device_may_wakeup(dev->otg.dev)) {
		enable_irq_wake(dev->irq);
		if (dev->vbus_on_irq)
			enable_irq_wake(dev->vbus_on_irq);
	}

	atomic_set(&dev->in_lpm, 1);

	if (!vbus && dev->pmic_notif_supp)
		dev->pdata->pmic_enable_ldo(0);

	pr_info("%s: usb in low power mode\n", __func__);

out:
	enable_irq(dev->irq);

	/* TBD: as there is no bus suspend implemented as of now
	 * it should be dummy check
	 */

	return 0;
}

static int msm_otg_resume(struct msm_otg *dev)
{
	unsigned temp;

	if (!atomic_read(&dev->in_lpm))
		return 0;


	if (dev->hs_pclk)
		clk_enable(dev->hs_pclk);
	if (dev->hs_cclk)
		clk_enable(dev->hs_cclk);

	temp = readl(USB_USBCMD);
	temp &= ~ASYNC_INTR_CTRL;
	temp &= ~ULPI_STP_CTRL;
	writel(temp, USB_USBCMD);

	/* If resume signalling finishes before lpm exit, PCD is not set in
	 * USBSTS register. Drive resume signal to the downstream device now
	 * so that host driver can process the upcoming port change interrupt.*/
	if (is_host())
		writel(readl(USB_PORTSC) | PORTSC_FPR, USB_PORTSC);

	if (device_may_wakeup(dev->otg.dev)) {
		disable_irq_wake(dev->irq);
		if (dev->vbus_on_irq)
			disable_irq_wake(dev->vbus_on_irq);
	}

	atomic_set(&dev->in_lpm, 0);

	pr_info("%s: usb exited from low power mode\n", __func__);

	return 0;
}

static int msm_otg_set_suspend(struct otg_transceiver *xceiv, int suspend)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	enum usb_otg_state state;
	unsigned long flags;

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	spin_lock_irqsave(&dev->lock, flags);
	state = dev->otg.state;
	spin_unlock_irqrestore(&dev->lock, flags);

	pr_debug("suspend request in state: %s\n",
			state_string(state));

	if (suspend) {
		switch (state) {
		case OTG_STATE_A_HOST:
			clear_bit(A_BUS_REQ, &dev->inputs);
			wake_lock(&dev->wlock);
			queue_work(dev->wq, &dev->sm_work);
			break;
		case OTG_STATE_B_PERIPHERAL:
			if (xceiv->gadget->b_hnp_enable) {
				set_bit(A_BUS_SUSPEND, &dev->inputs);
				set_bit(B_BUS_REQ, &dev->inputs);
				wake_lock(&dev->wlock);
				queue_work(dev->wq, &dev->sm_work);
			}
			break;
		case OTG_STATE_A_PERIPHERAL:
			msm_otg_start_timer(dev, TA_BIDL_ADIS,
					A_BIDL_ADIS);
			break;
		default:
			break;
		}
	} else {
		unsigned long timeout;

		switch (state) {
		case OTG_STATE_A_PERIPHERAL:
			/* A-peripheral observed activity on bus.
			 * clear A_BIDL_ADIS timer.
			 */
			msm_otg_del_timer(dev);
			break;
		case OTG_STATE_A_SUSPEND:
			/* Remote wakeup or resume */
			set_bit(A_BUS_REQ, &dev->inputs);
			spin_lock_irqsave(&dev->lock, flags);
			dev->otg.state = OTG_STATE_A_HOST;
			spin_unlock_irqrestore(&dev->lock, flags);
			break;
		default:
			break;
		}

		if (suspend == atomic_read(&dev->in_lpm))
			return 0;

		disable_irq(dev->irq);
		if (dev->pmic_notif_supp)
			dev->pdata->pmic_enable_ldo(1);

		msm_otg_resume(dev);

		if (!is_phy_clk_disabled())
			goto out;

		timeout = jiffies + usecs_to_jiffies(100);
		enable_phy_clk();
		while (is_phy_clk_disabled()) {
			if (time_after(jiffies, timeout)) {
				pr_err("%s: Unable to wakeup phy\n", __func__);
				/* Reset both phy and link */
				otg_reset(&dev->otg, 1);
				break;
			}
			udelay(10);
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
		if (dev->pmic_notif_supp && dev->pdata->pmic_unregister_vbus_sn)
			dev->pdata->pmic_unregister_vbus_sn
				(&msm_otg_set_vbus_state);
		return 0;
	}
	dev->otg.gadget = gadget;
	enable_sess_valid(dev);
	if (dev->pmic_notif_supp && dev->pdata->pmic_register_vbus_sn)
		dev->pdata->pmic_register_vbus_sn(&msm_otg_set_vbus_state);
	pr_info("peripheral driver registered w/ tranceiver\n");

	wake_lock(&dev->wlock);
	queue_work(dev->wq, &dev->sm_work);
	return 0;
}

#ifdef CONFIG_USB_EHCI_MSM
static int usbdev_notify(struct notifier_block *self,
			unsigned long action, void *device)
{
	enum usb_otg_state state;
	struct msm_otg *dev = container_of(self, struct msm_otg, usbdev_nb);
	struct usb_device *udev = device;
	int work = 1;

	/* Interested in only devices directly connected
	 * to root hub directly.
	 */
	if (!udev->parent || udev->parent->parent)
		goto out;

	spin_lock_irq(&dev->lock);
	state = dev->otg.state;
	spin_unlock_irq(&dev->lock);

	switch (state) {
	case OTG_STATE_A_WAIT_BCON:
		if (action == USB_DEVICE_ADD) {
			pr_debug("B_CONN set\n");
			set_bit(B_CONN, &dev->inputs);
		}
		break;
	case OTG_STATE_A_HOST:
		if (action == USB_DEVICE_REMOVE) {
			pr_debug("B_CONN clear\n");
			clear_bit(B_CONN, &dev->inputs);
		}
		break;
	default:
		work = 0;
		break;
	}

	if (work) {
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}
out:
	return NOTIFY_OK;
}

static int msm_otg_set_host(struct otg_transceiver *xceiv, struct usb_bus *host)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	/* Id pin is not routed to PMIC. Host mode can not be
	 * supported with pmic notification support.
	 */
	if (!dev->start_host || dev->pmic_notif_supp)
		return -ENODEV;

	if (!host) {
		msm_otg_start_host(xceiv, REQUEST_STOP);
		usb_unregister_notify(&dev->usbdev_nb);
		dev->otg.host = 0;
		dev->start_host = 0;
		disable_idgnd(dev);
		return 0;
	}
#ifdef CONFIG_USB_OTG
	host->otg_port = 1;
#endif
	dev->usbdev_nb.notifier_call = usbdev_notify;
	usb_register_notify(&dev->usbdev_nb);
	dev->otg.host = host;
	enable_idgnd(dev);
	pr_info("host driver registered w/ tranceiver\n");

#ifndef CONFIG_USB_MSM_72K
	wake_lock(&dev->wlock);
	queue_work(dev->wq, &dev->sm_work);
#endif
	return 0;
}
#endif

static void msm_otg_set_vbus_state(int online)
{
	struct msm_otg *dev = the_msm_otg;

	if (online)
		msm_otg_set_suspend(&dev->otg, 0);
}

/* pmic irq handlers are called from thread context and
 * are allowed to sleep
 */
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	struct msm_otg *dev = the_msm_otg;

	if (!dev->otg.gadget)
		return IRQ_HANDLED;

	pr_info("%s: vbus notification from pmic\n", __func__);

	msm_otg_set_suspend(&dev->otg, 0);

	return IRQ_HANDLED;
}

static irqreturn_t msm_otg_irq(int irq, void *data)
{
	struct msm_otg *dev = data;
	u32 otgsc, sts, pc, sts_mask;
	irqreturn_t ret = IRQ_HANDLED;
	int work = 0;
	enum usb_otg_state state;

	if (atomic_read(&dev->in_lpm)) {
		msm_otg_resume(dev);
		goto out;
	}

	/* Return immediately if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return IRQ_NONE;

	otgsc = readl(USB_OTGSC);
	sts = readl(USB_USBSTS);

	sts_mask = (otgsc & OTGSC_INTR_MASK) >> 8;

	if (!((otgsc & sts_mask) || (sts & STS_PCI))) {
		ret = IRQ_NONE;
		goto out;
	}

	spin_lock(&dev->lock);
	state = dev->otg.state;
	spin_unlock(&dev->lock);

	pr_debug("IRQ state: %s\n", state_string(state));
	pr_debug("otgsc = %x\n", otgsc);

	if (otgsc & OTGSC_IDIS) {
		if (otgsc & OTGSC_ID) {
			pr_debug("Id set\n");
			set_bit(ID, &dev->inputs);
		} else {
			pr_debug("Id clear\n");
			/* Assert a_bus_req to supply power on
			 * VBUS when Micro/Mini-A cable is connected
			 * with out user intervention.
			 */
			set_bit(A_BUS_REQ, &dev->inputs);
			clear_bit(ID, &dev->inputs);
		}
		writel(otgsc, USB_OTGSC);
		work = 1;
	} else if (otgsc & OTGSC_BSVIS) {
		writel(otgsc, USB_OTGSC);
		/* BSV interrupt comes when operating as an A-device
		 * (VBUS on/off).
		 */
		if (state >= OTG_STATE_A_IDLE)
			goto out;
		if (otgsc & OTGSC_BSV) {
			pr_debug("BSV set\n");
			set_bit(B_SESS_VLD, &dev->inputs);
		} else {
			pr_debug("BSV clear\n");
			clear_bit(B_SESS_VLD, &dev->inputs);
		}
		work = 1;
	} else if (otgsc & OTGSC_DPIS) {
		pr_debug("DPIS detected\n");
		writel(otgsc, USB_OTGSC);
		set_bit(A_SRP_DET, &dev->inputs);
		set_bit(A_BUS_REQ, &dev->inputs);
		work = 1;
	} else if (sts & STS_PCI) {
		pc = readl(USB_PORTSC);
		pr_debug("portsc = %x\n", pc);
		ret = IRQ_NONE;
		/* HCD Acks PCI interrupt. We use this to switch
		 * between different OTG states.
		 */
		work = 1;
		switch (state) {
		case OTG_STATE_A_SUSPEND:
			if (dev->otg.host->b_hnp_enable && (pc & PORTSC_CSC) &&
					!(pc & PORTSC_CCS)) {
				pr_debug("B_CONN clear\n");
				clear_bit(B_CONN, &dev->inputs);
			}
			break;
		case OTG_STATE_B_WAIT_ACON:
			if ((pc & PORTSC_CSC) && (pc & PORTSC_CCS)) {
				pr_debug("A_CONN set\n");
				set_bit(A_CONN, &dev->inputs);
				/* Clear ASE0_BRST timer */
				msm_otg_del_timer(dev);
			}
			break;
		case OTG_STATE_B_HOST:
			if ((pc & PORTSC_CSC) && !(pc & PORTSC_CCS)) {
				pr_debug("A_CONN clear\n");
				clear_bit(A_CONN, &dev->inputs);
			}
			break;
		default:
			work = 0;
			break;
		}
	}
	if (work) {
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}
out:
	return ret;
}

#define ULPI_VERIFY_MAX_LOOP_COUNT  5
#define PHY_CALIB_RETRY_COUNT 10
static void phy_clk_reset(struct msm_otg *dev)
{
	unsigned rc;
	enum clk_reset_action assert = CLK_RESET_ASSERT;

	if (dev->pdata->phy_reset_sig_inverted)
		assert = CLK_RESET_DEASSERT;

	rc = clk_reset(dev->phy_reset_clk, assert);
	if (rc) {
		pr_err("%s: phy clk assert failed\n", __func__);
		return;
	}

	msleep(1);

	rc = clk_reset(dev->phy_reset_clk, !assert);
	if (rc) {
		pr_err("%s: phy clk deassert failed\n", __func__);
		return;
	}

	msleep(1);
}

static unsigned ulpi_read_with_reset(struct msm_otg *dev, unsigned reg)
{
	int temp;
	unsigned res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_read(dev, reg);
		if (res != 0xffffffff)
			return res;

		phy_clk_reset(dev);
	}

	pr_err("%s: ulpi read failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

static int ulpi_write_with_reset(struct msm_otg *dev,
unsigned val, unsigned reg)
{
	int temp, res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_write(dev, val, reg);
		if (!res)
			return 0;
		phy_clk_reset(dev);
	}
	pr_err("%s: ulpi write failed for %d times\n",
		__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

/* some of the older targets does not turn off the PLL
 * if onclock bit is set and clocksuspendM bit is on,
 * hence clear them too and initiate the suspend mode
 * by clearing SupendM bit.
 */
static inline int turn_off_phy_pll(struct msm_otg *dev)
{
	unsigned res;

	res = ulpi_read_with_reset(dev, ULPI_CONFIG_REG1);
	if (res == 0xffffffff)
		return -ETIMEDOUT;

	res = ulpi_write_with_reset(dev,
		res & ~(ULPI_ONCLOCK), ULPI_CONFIG_REG1);
	if (res)
		return -ETIMEDOUT;

	res = ulpi_write_with_reset(dev,
		ULPI_CLOCK_SUSPENDM, ULPI_IFC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	/*Clear SuspendM bit to initiate suspend mode */
	res = ulpi_write_with_reset(dev,
		ULPI_SUSPENDM, ULPI_FUNC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	return res;
}

static inline int check_phy_caliberation(struct msm_otg *dev)
{
	unsigned res;

	res = ulpi_read_with_reset(dev, ULPI_DEBUG);

	if (res == 0xffffffff)
		return -ETIMEDOUT;

	if (!(res & ULPI_CALIB_STS) && ULPI_CALIB_VAL(res))
		return 0;

	return -1;
}

static int msm_otg_phy_caliberate(struct msm_otg *dev)
{
	int i = 0;
	unsigned long res;

	do {
		res = turn_off_phy_pll(dev);
		if (res)
			return -ETIMEDOUT;

		/* bring phy out of suspend */
		phy_clk_reset(dev);

		res = check_phy_caliberation(dev);
		if (!res)
			return res;
		i++;

	} while (i < PHY_CALIB_RETRY_COUNT);

	return res;
}

static int msm_otg_phy_reset(struct msm_otg *dev)
{
	unsigned rc;
	unsigned temp;
	unsigned long timeout;

	rc = clk_reset(dev->hs_clk, CLK_RESET_ASSERT);
	if (rc) {
		pr_err("%s: usb hs clk assert failed\n", __func__);
		return -1;
	}

	phy_clk_reset(dev);

	rc = clk_reset(dev->hs_clk, CLK_RESET_DEASSERT);
	if (rc) {
		pr_err("%s: usb hs clk deassert failed\n", __func__);
		return -1;
	}

	/* select ULPI phy */
	temp = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

	rc = msm_otg_phy_caliberate(dev);
	if (rc)
		return rc;

	/* TBD: There are two link resets. One is below and other one
	 * is done immediately after this function. See if we can
	 * eliminate one of these.
	 */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	do {
		if (time_after(jiffies, timeout)) {
			pr_err("msm_otg: usb link reset timeout\n");
			break;
		}
		msleep(1);
	} while (readl(USB_USBCMD) & USBCMD_RESET);

	if (readl(USB_USBCMD) & USBCMD_RESET) {
		pr_err("%s: usb core reset failed\n", __func__);
		return -1;
	}

	return 0;
}

static void otg_reset(struct otg_transceiver *xceiv, int phy_reset)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	unsigned long timeout;
	u32 mode;

	clk_enable(dev->hs_clk);

	if (!phy_reset)
		goto reset_link;

	if (dev->pdata->phy_reset)
		dev->pdata->phy_reset(dev->regs);
	else
		msm_otg_phy_reset(dev);

	/*disable all phy interrupts*/
	ulpi_write(dev, 0xFF, 0x0F);
	ulpi_write(dev, 0xFF, 0x12);
	msleep(100);

reset_link:
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	do {
		if (time_after(jiffies, timeout)) {
			pr_err("msm_otg: usb link reset timeout\n");
			break;
		}
		msleep(1);
	} while (readl(USB_USBCMD) & USBCMD_RESET);

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	set_pre_emphasis_level(dev);
	set_cdr_auto_reset(dev);
	set_driver_amplitude(dev);

	writel(0x0, USB_AHB_BURST);
	writel(0x00, USB_AHB_MODE);
	clk_disable(dev->hs_clk);

	if (test_bit(ID, &dev->inputs))
		mode = USBMODE_SDIS | USBMODE_DEVICE;
	else
		mode = USBMODE_SDIS | USBMODE_HOST;
	writel(mode, USB_USBMODE);

	if (dev->otg.gadget)
		enable_sess_valid(dev);
#ifdef CONFIG_USB_EHCI_MSM
	if (dev->otg.host)
		enable_idgnd(dev);
#endif
}

static void msm_otg_sm_work(struct work_struct *w)
{
	struct msm_otg *dev = container_of(w, struct msm_otg, sm_work);
	int ret;
	int work = 0;
	enum usb_otg_state state;

	if (atomic_read(&dev->in_lpm))
		msm_otg_set_suspend(&dev->otg, 0);

	spin_lock_irq(&dev->lock);
	state = dev->otg.state;
	spin_unlock_irq(&dev->lock);

	pr_debug("state: %s\n", state_string(state));

	switch (state) {
	case OTG_STATE_UNDEFINED:
		if  (!dev->otg.host || !is_host() ||
				(dev->pdata->otg_mode == OTG_USER_CONTROL))
			set_bit(ID, &dev->inputs);

		if ((dev->otg.gadget && is_b_sess_vld()) ||
				(dev->pdata->otg_mode == OTG_USER_CONTROL))
			set_bit(B_SESS_VLD, &dev->inputs);

		spin_lock_irq(&dev->lock);
		if (test_bit(ID, &dev->inputs)) {
			dev->otg.state = OTG_STATE_B_IDLE;
		} else {
			set_bit(A_BUS_REQ, &dev->inputs);
			dev->otg.state = OTG_STATE_A_IDLE;
		}
		spin_unlock_irq(&dev->lock);

		work = 1;
		break;
	case OTG_STATE_B_IDLE:
		dev->otg.default_a = 0;
		if (!test_bit(ID, &dev->inputs)) {
			pr_debug("!id\n");
			clear_bit(B_BUS_REQ, &dev->inputs);
			otg_reset(&dev->otg, 0);

			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_IDLE;
			spin_unlock_irq(&dev->lock);
			work = 1;
		} else if (test_bit(B_SESS_VLD, &dev->inputs)) {
			pr_debug("b_sess_vld\n");
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_PERIPHERAL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_peripheral(&dev->otg, 1);
		} else if (test_bit(B_BUS_REQ, &dev->inputs)) {
			pr_debug("b_sess_end && b_bus_req\n");
			ret = msm_otg_start_srp(&dev->otg);
			if (ret < 0) {
				/* notify user space */
				clear_bit(B_BUS_REQ, &dev->inputs);
				work = 1;
				break;
			}
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_SRP_INIT;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_timer(dev, TB_SRP_FAIL, B_SRP_FAIL);
			break;
		} else {
			pr_debug("entering into lpm\n");
			msm_otg_suspend(dev);

		}
		break;
	case OTG_STATE_B_SRP_INIT:
		if (!test_bit(ID, &dev->inputs) ||
				test_bit(B_SESS_VLD, &dev->inputs)) {
			pr_debug("!id || b_sess_vld\n");
			msm_otg_del_timer(dev);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_IDLE;
			spin_unlock_irq(&dev->lock);
			work = 1;
		} else if (test_bit(B_SRP_FAIL, &dev->tmouts)) {
			pr_debug("b_srp_fail\n");
			/* notify user space */
			msm_otg_send_event(&dev->otg,
				OTG_EVENT_NO_RESP_FOR_SRP);
			clear_bit(B_BUS_REQ, &dev->inputs);
			clear_bit(B_SRP_FAIL, &dev->tmouts);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_IDLE;
			spin_unlock_irq(&dev->lock);
			dev->b_last_se0_sess = jiffies;
			work = 1;
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!test_bit(ID, &dev->inputs) ||
				!test_bit(B_SESS_VLD, &dev->inputs)) {
			pr_debug("!id || !b_sess_vld\n");
			clear_bit(B_BUS_REQ, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_IDLE;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_peripheral(&dev->otg, 0);
			dev->b_last_se0_sess = jiffies;

			/* Workaround: Reset phy after session */
			otg_reset(&dev->otg, 1);

			/* come back later to put hardware in
			 * lpm. This removes addition checks in
			 * suspend routine for missing BSV
			 */
			work = 1;
		} else if (test_bit(B_BUS_REQ, &dev->inputs) &&
				dev->otg.gadget->b_hnp_enable &&
				test_bit(A_BUS_SUSPEND, &dev->inputs)) {
			pr_debug("b_bus_req && b_hnp_en && a_bus_suspend\n");
			msm_otg_start_timer(dev, TB_ASE0_BRST, B_ASE0_BRST);
			msm_otg_start_peripheral(&dev->otg, 0);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_WAIT_ACON;
			spin_unlock_irq(&dev->lock);
			/* start HCD even before A-device enable
			 * pull-up to meet HNP timings.
			 */
			dev->otg.host->is_b_host = 1;
			msm_otg_start_host(&dev->otg, REQUEST_START);

		}
		break;
	case OTG_STATE_B_WAIT_ACON:
		if (!test_bit(ID, &dev->inputs) ||
				!test_bit(B_SESS_VLD, &dev->inputs)) {
			pr_debug("!id || !b_sess_vld\n");
			msm_otg_del_timer(dev);
			/* A-device is physically disconnected during
			 * HNP. Remove HCD.
			 */
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			dev->otg.host->is_b_host = 0;

			clear_bit(B_BUS_REQ, &dev->inputs);
			clear_bit(A_BUS_SUSPEND, &dev->inputs);
			dev->b_last_se0_sess = jiffies;
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_IDLE;
			spin_unlock_irq(&dev->lock);

			/* Workaround: Reset phy after session */
			otg_reset(&dev->otg, 1);
			work = 1;
		} else if (test_bit(A_CONN, &dev->inputs)) {
			pr_debug("a_conn\n");
			clear_bit(A_BUS_SUSPEND, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_HOST;
			spin_unlock_irq(&dev->lock);
		} else if (test_bit(B_ASE0_BRST, &dev->tmouts)) {
			/* TODO: A-device may send reset after
			 * enabling HNP; a_bus_resume case is
			 * not handled for now.
			 */
			pr_debug("b_ase0_brst_tmout\n");
			msm_otg_send_event(&dev->otg,
				OTG_EVENT_HNP_FAILED);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			dev->otg.host->is_b_host = 0;
			clear_bit(B_ASE0_BRST, &dev->tmouts);
			clear_bit(A_BUS_SUSPEND, &dev->inputs);
			clear_bit(B_BUS_REQ, &dev->inputs);

			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_PERIPHERAL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_peripheral(&dev->otg, 1);
		}
		break;
	case OTG_STATE_B_HOST:
		/* B_BUS_REQ is not exposed to user space. So
		 * it must be A_CONN for now.
		 */
		if (!test_bit(B_BUS_REQ, &dev->inputs) ||
				!test_bit(A_CONN, &dev->inputs)) {
			pr_debug("!b_bus_req || !a_conn\n");
			clear_bit(A_CONN, &dev->inputs);
			clear_bit(B_BUS_REQ, &dev->inputs);

			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			dev->otg.host->is_b_host = 0;

			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_IDLE;
			spin_unlock_irq(&dev->lock);
			/* Workaround: Reset phy after session */
			otg_reset(&dev->otg, 1);
			work = 1;
		}
		break;
	case OTG_STATE_A_IDLE:
		dev->otg.default_a = 1;
		if (test_bit(ID, &dev->inputs)) {
			pr_debug("id\n");
			dev->otg.default_a = 0;
			otg_reset(&dev->otg, 0);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_B_IDLE;
			spin_unlock_irq(&dev->lock);
			work = 1;
		} else if (!test_bit(A_BUS_DROP, &dev->inputs) &&
				(test_bit(A_SRP_DET, &dev->inputs) ||
				 test_bit(A_BUS_REQ, &dev->inputs))) {
			pr_debug("!a_bus_drop && (a_srp_det || a_bus_req)\n");

			clear_bit(A_SRP_DET, &dev->inputs);
			/* Disable SRP detection */
			writel((readl(USB_OTGSC) & ~OTGSC_INTR_STS_MASK) &
					~OTGSC_DPIE, USB_OTGSC);

			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VRISE;
			spin_unlock_irq(&dev->lock);
			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 1);
			msm_otg_start_timer(dev, TA_WAIT_VRISE, A_WAIT_VRISE);
			/* no need to schedule work now */
		} else {
			pr_debug("No session requested\n");

			/* A-device is not providing power on VBUS.
			 * Enable SRP detection.
			 */
			writel((readl(USB_OTGSC) & ~OTGSC_INTR_STS_MASK) |
					OTGSC_DPIE, USB_OTGSC);
			msm_otg_suspend(dev);

		}
		break;
	case OTG_STATE_A_WAIT_VRISE:
		if (test_bit(ID, &dev->inputs) ||
				test_bit(A_BUS_DROP, &dev->inputs) ||
				test_bit(A_WAIT_VRISE, &dev->tmouts)) {
			pr_debug("id || a_bus_drop || a_wait_vrise_tmout\n");
			clear_bit(A_BUS_REQ, &dev->inputs);
			msm_otg_del_timer(dev);
			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 0);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VFALL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_timer(dev, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (test_bit(A_VBUS_VLD, &dev->inputs)) {
			pr_debug("a_vbus_vld\n");
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_BCON;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_timer(dev, TA_WAIT_BCON, A_WAIT_BCON);
			/* Start HCD to detect peripherals. */
			msm_otg_start_host(&dev->otg, REQUEST_START);
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (test_bit(ID, &dev->inputs) ||
				test_bit(A_BUS_DROP, &dev->inputs) ||
				test_bit(A_WAIT_BCON, &dev->tmouts)) {
			pr_debug("id || a_bus_drop || a_wait_bcon_tmout\n");
			if (test_bit(A_WAIT_BCON, &dev->tmouts))
				msm_otg_send_event(&dev->otg,
					OTG_EVENT_DEV_CONN_TMOUT);
			msm_otg_del_timer(dev);
			clear_bit(A_BUS_REQ, &dev->inputs);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 0);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VFALL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_timer(dev, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (test_bit(B_CONN, &dev->inputs)) {
			pr_debug("b_conn\n");
			msm_otg_del_timer(dev);
			/* HCD is added already. just move to
			 * A_HOST state.
			 */
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_HOST;
			spin_unlock_irq(&dev->lock);
		} else if (!test_bit(A_VBUS_VLD, &dev->inputs)) {
			pr_debug("!a_vbus_vld\n");
			msm_otg_del_timer(dev);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_VBUS_ERR;
			spin_unlock_irq(&dev->lock);
		}
		break;
	case OTG_STATE_A_HOST:
		if (test_bit(ID, &dev->inputs) ||
				test_bit(A_BUS_DROP, &dev->inputs)) {
			pr_debug("id || a_bus_drop\n");
			clear_bit(B_CONN, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VFALL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 0);
			msm_otg_start_timer(dev, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &dev->inputs)) {
			pr_debug("!a_vbus_vld\n");
			clear_bit(B_CONN, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_VBUS_ERR;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			/* no work */
		} else if (!test_bit(A_BUS_REQ, &dev->inputs)) {
			/* a_bus_req is de-asserted when root hub is
			 * suspended or HNP is in progress.
			 */
			pr_debug("!a_bus_req\n");
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_SUSPEND;
			spin_unlock_irq(&dev->lock);
			if (dev->otg.host->b_hnp_enable) {
				msm_otg_start_timer(dev, TA_AIDL_BDIS,
						A_AIDL_BDIS);
			} else {
				/* No HNP. Root hub suspended */
				msm_otg_suspend(dev);
			}
		} else if (!test_bit(B_CONN, &dev->inputs)) {
			pr_debug("!b_conn\n");
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_BCON;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_timer(dev, TA_WAIT_BCON, A_WAIT_BCON);
		}
		break;
	case OTG_STATE_A_SUSPEND:
		if (test_bit(ID, &dev->inputs) ||
				test_bit(A_BUS_DROP, &dev->inputs) ||
				test_bit(A_AIDL_BDIS, &dev->tmouts)) {
			pr_debug("id || a_bus_drop || a_aidl_bdis_tmout\n");
			if (test_bit(A_AIDL_BDIS, &dev->tmouts))
				msm_otg_send_event(&dev->otg,
					OTG_EVENT_HNP_FAILED);
			msm_otg_del_timer(dev);
			clear_bit(B_CONN, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VFALL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 0);
			msm_otg_start_timer(dev, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &dev->inputs)) {
			pr_debug("!a_vbus_vld\n");
			msm_otg_del_timer(dev);
			clear_bit(B_CONN, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_VBUS_ERR;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
		} else if (!test_bit(B_CONN, &dev->inputs) &&
				dev->otg.host->b_hnp_enable) {
			pr_debug("!b_conn && b_hnp_enable");
			/* Clear AIDL_BDIS timer */
			msm_otg_del_timer(dev);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_PERIPHERAL;
			spin_unlock_irq(&dev->lock);

			msm_otg_start_host(&dev->otg, REQUEST_HNP_SUSPEND);

			/* We may come here even when B-dev is physically
			 * disconnected during HNP. We go back to host
			 * role if bus is idle for BIDL_ADIS time.
			 */
			dev->otg.gadget->is_a_peripheral = 1;
			msm_otg_start_peripheral(&dev->otg, 1);
		} else if (!test_bit(B_CONN, &dev->inputs) &&
				!dev->otg.host->b_hnp_enable) {
			pr_debug("!b_conn && !b_hnp_enable");
			/* bus request is dropped during suspend.
			 * acquire again for next device.
			 */
			set_bit(A_BUS_REQ, &dev->inputs);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_BCON;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_timer(dev, TA_WAIT_BCON, A_WAIT_BCON);
		}
		break;
	case OTG_STATE_A_PERIPHERAL:
		if (test_bit(ID, &dev->inputs) ||
				test_bit(A_BUS_DROP, &dev->inputs)) {
			pr_debug("id || a_bus_drop\n");
			/* Clear BIDL_ADIS timer */
			msm_otg_del_timer(dev);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VFALL;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_peripheral(&dev->otg, 0);
			dev->otg.gadget->is_a_peripheral = 0;
			/* HCD was suspended before. Stop it now */
			msm_otg_start_host(&dev->otg, REQUEST_STOP);

			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 0);
			msm_otg_start_timer(dev, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &dev->inputs)) {
			pr_debug("!a_vbus_vld\n");
			/* Clear BIDL_ADIS timer */
			msm_otg_del_timer(dev);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_VBUS_ERR;
			spin_unlock_irq(&dev->lock);
			msm_otg_start_peripheral(&dev->otg, 0);
			dev->otg.gadget->is_a_peripheral = 0;
			/* HCD was suspended before. Stop it now */
			msm_otg_start_host(&dev->otg, REQUEST_STOP);
		} else if (test_bit(A_BIDL_ADIS, &dev->tmouts)) {
			pr_debug("a_bidl_adis_tmout\n");
			msm_otg_start_peripheral(&dev->otg, 0);
			dev->otg.gadget->is_a_peripheral = 0;

			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_BCON;
			spin_unlock_irq(&dev->lock);
			set_bit(A_BUS_REQ, &dev->inputs);
			msm_otg_start_host(&dev->otg, REQUEST_HNP_RESUME);
			msm_otg_start_timer(dev, TA_WAIT_BCON, A_WAIT_BCON);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		if (test_bit(A_WAIT_VFALL, &dev->tmouts)) {
			clear_bit(A_VBUS_VLD, &dev->inputs);
			/* Reset both phy and link */
			otg_reset(&dev->otg, 1);
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_IDLE;
			spin_unlock_irq(&dev->lock);
			work = 1;
		}
		break;
	case OTG_STATE_A_VBUS_ERR:
		if (test_bit(ID, &dev->inputs) ||
				test_bit(A_BUS_DROP, &dev->inputs) ||
				test_bit(A_CLR_ERR, &dev->inputs)) {
			spin_lock_irq(&dev->lock);
			dev->otg.state = OTG_STATE_A_WAIT_VFALL;
			spin_unlock_irq(&dev->lock);
			dev->pdata->vbus_power(USB_PHY_INTEGRATED, 0);
			msm_otg_start_timer(dev, TA_WAIT_VFALL, A_WAIT_VFALL);
		}
		break;
	default:
		pr_err("invalid OTG state\n");
	}

	if (work)
		queue_work(dev->wq, &dev->sm_work);

	/* IRQ/sysfs may queue work. Check work_pending. otherwise
	 * we might endup releasing wakelock after it is acquired
	 * in IRQ/sysfs.
	 */
	if (!work_pending(&dev->sm_work) && !hrtimer_active(&dev->timer))
		wake_unlock(&dev->wlock);
}

static ssize_t
set_pwr_down(struct device *_dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct msm_otg *dev = the_msm_otg;
	int value;
	enum usb_otg_state state;

	spin_lock_irq(&dev->lock);
	state = dev->otg.state;
	spin_unlock_irq(&dev->lock);

	/* Applicable for only A-Device */
	if (state <= OTG_STATE_A_IDLE)
		return -EINVAL;

	sscanf(buf, "%d", &value);

	if (test_bit(A_BUS_DROP, &dev->inputs) != !!value) {
		change_bit(A_BUS_DROP, &dev->inputs);
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}

	return count;
}
static DEVICE_ATTR(pwr_down, S_IRUGO | S_IWUSR, NULL, set_pwr_down);

static ssize_t
set_srp_req(struct device *_dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct msm_otg *dev = the_msm_otg;
	enum usb_otg_state state;

	spin_lock_irq(&dev->lock);
	state = dev->otg.state;
	spin_unlock_irq(&dev->lock);

	if (state != OTG_STATE_B_IDLE)
		return -EINVAL;

	set_bit(B_BUS_REQ, &dev->inputs);
	wake_lock(&dev->wlock);
	queue_work(dev->wq, &dev->sm_work);

	return count;
}
static DEVICE_ATTR(srp_req, S_IRUGO | S_IWUSR, NULL, set_srp_req);

static ssize_t
set_clr_err(struct device *_dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct msm_otg *dev = the_msm_otg;
	enum usb_otg_state state;

	spin_lock_irq(&dev->lock);
	state = dev->otg.state;
	spin_unlock_irq(&dev->lock);

	if (state == OTG_STATE_A_VBUS_ERR) {
		set_bit(A_CLR_ERR, &dev->inputs);
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}

	return count;
}
static DEVICE_ATTR(clr_err, S_IRUGO | S_IWUSR, NULL, set_clr_err);

static struct attribute *msm_otg_attrs[] = {
	&dev_attr_pwr_down.attr,
	&dev_attr_srp_req.attr,
	&dev_attr_clr_err.attr,
	NULL,
};

static struct attribute_group msm_otg_attr_grp = {
	.attrs = msm_otg_attrs,
};

#ifdef CONFIG_DEBUG_FS
static int otg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t otg_mode_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct msm_otg *dev = file->private_data;
	int ret = count;
	int work = 0;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	if (!memcmp(buf, "none", count - 1)) {
		clear_bit(B_SESS_VLD, &dev->inputs);
		set_bit(ID, &dev->inputs);
		work = 1;
	} else if (!memcmp(buf, "peripheral", count - 1)) {
		set_bit(B_SESS_VLD, &dev->inputs);
		set_bit(ID, &dev->inputs);
		work = 1;
	} else if (!memcmp(buf, "host", count - 1)) {
		clear_bit(B_SESS_VLD, &dev->inputs);
		clear_bit(ID, &dev->inputs);
		set_bit(A_BUS_REQ, &dev->inputs);
		work = 1;
	} else {
		pr_info("%s: unknown mode specified\n", __func__);
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	if (work) {
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}

	return ret;
}
const struct file_operations otgfs_fops = {
	.open	= otg_open,
	.write	= otg_mode_write,
};

struct dentry *otg_debug_root;
struct dentry *otg_debug_mode;

static int otg_debugfs_init(struct msm_otg *dev)
{
	otg_debug_root = debugfs_create_dir("otg", NULL);
	if (!otg_debug_root)
		return -ENOENT;

	otg_debug_mode = debugfs_create_file("mode", 0222,
						otg_debug_root, dev,
						&otgfs_fops);
	if (!otg_debug_mode) {
		debugfs_remove(otg_debug_root);
		otg_debug_root = NULL;
		return -ENOENT;
	}

	return 0;
}

static void otg_debugfs_cleanup(void)
{
       debugfs_remove(otg_debug_mode);
       debugfs_remove(otg_debug_root);
}
#endif
static int __init msm_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	int vbus_on_irq = 0;
	struct resource *res;
	struct msm_otg *dev;

	dev = kzalloc(sizeof(struct msm_otg), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->otg.dev = &pdev->dev;
	dev->pdata = pdev->dev.platform_data;

	if (!dev->pdata) {
		ret = -ENODEV;
		goto free_dev;
	}

#ifdef CONFIG_USB_EHCI_MSM
	if (!dev->pdata->vbus_power) {
		ret = -ENODEV;
		goto free_dev;
	}
#endif

	if (dev->pdata->pmic_vbus_irq) {
		vbus_on_irq = platform_get_irq_byname(pdev, "vbus_on");
		if (vbus_on_irq < 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			ret = vbus_on_irq;
			goto free_dev;
		}
	}

	if (dev->pdata->rpc_connect) {
		ret = dev->pdata->rpc_connect(1);
		pr_info("%s: rpc_connect(%d)\n", __func__, ret);
		if (ret) {
			pr_err("%s: rpc connect failed\n", __func__);
			ret = -ENODEV;
			goto free_dev;
		}
	}

	dev->hs_clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(dev->hs_clk)) {
		pr_err("%s: failed to get usb_hs_clk\n", __func__);
		ret = PTR_ERR(dev->hs_clk);
		goto rpc_fail;
	}
	clk_set_rate(dev->hs_clk, 60000000);

	if (!dev->pdata->usb_in_sps) {
		dev->hs_pclk = clk_get(&pdev->dev, "usb_hs_pclk");
		if (IS_ERR(dev->hs_pclk)) {
			pr_err("%s: failed to get usb_hs_pclk\n", __func__);
			ret = PTR_ERR(dev->hs_pclk);
			goto put_hs_clk;
		}
	}

	if (dev->pdata->core_clk) {
		dev->hs_cclk = clk_get(&pdev->dev, "usb_hs_core_clk");
		if (IS_ERR(dev->hs_cclk)) {
			pr_err("%s: failed to get usb_hs_core_clk\n", __func__);
			ret = PTR_ERR(dev->hs_cclk);
			goto put_hs_pclk;
		}
	}

	if (!dev->pdata->phy_reset) {
		dev->phy_reset_clk = clk_get(&pdev->dev, "usb_phy_clk");
		if (IS_ERR(dev->phy_reset_clk)) {
			pr_err("%s: failed to get usb_phy_clk\n", __func__);
			ret = PTR_ERR(dev->phy_reset_clk);
			goto put_hs_cclk;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("%s: failed to get platform resource mem\n", __func__);
		ret = -ENODEV;
		goto put_phy_clk;
	}

	dev->regs = ioremap(res->start, resource_size(res));
	if (!dev->regs) {
		pr_err("%s: ioremap failed\n", __func__);
		ret = -ENOMEM;
		goto put_phy_clk;
	}
	dev->irq = platform_get_irq(pdev, 0);
	if (!dev->irq) {
		pr_err("%s: platform_get_irq failed\n", __func__);
		ret = -ENODEV;
		goto free_regs;
	}

	msm_otg_init_timer(dev);
	INIT_WORK(&dev->sm_work, msm_otg_sm_work);
	spin_lock_init(&dev->lock);
	wake_lock_init(&dev->wlock, WAKE_LOCK_SUSPEND, "msm_otg");

	dev->wq = create_singlethread_workqueue("k_otg");
	if (!dev->wq) {
		ret = -ENOMEM;
		goto free_wlock;
	}

	/* enable clocks */
	if (dev->hs_pclk)
		clk_enable(dev->hs_pclk);
	if (dev->hs_cclk)
		clk_enable(dev->hs_cclk);

	/* To reduce phy power consumption and to avoid external LDO
	 * on the board, PMIC comparators can be used to detect VBUS
	 * session change.
	 */
	if (dev->pdata->pmic_notif_init) {
		ret = dev->pdata->pmic_notif_init();
		if (!ret) {
			dev->pmic_notif_supp = 1;
			dev->pdata->pmic_enable_ldo(1);
		} else if (ret != -ENOTSUPP) {
			if (dev->hs_pclk)
				clk_disable(dev->hs_pclk);
			if (dev->hs_cclk)
				clk_disable(dev->hs_cclk);
			goto free_wq;
		}
	}

	/* Reset both phy and link */
	otg_reset(&dev->otg, 1);

	/* ACk all pending interrupts and clear interrupt enable registers */
	writel((readl(USB_OTGSC) & ~OTGSC_INTR_MASK), USB_OTGSC);
	writel(readl(USB_USBSTS), USB_USBSTS);
	writel(0, USB_USBINTR);

	ret = request_irq(dev->irq, msm_otg_irq, IRQF_SHARED,
					"msm_otg", dev);
	if (ret) {
		pr_info("%s: request irq failed\n", __func__);
		if (dev->hs_pclk)
			clk_disable(dev->hs_pclk);
		if (dev->hs_cclk)
			clk_disable(dev->hs_cclk);
		goto free_wq;
	}

	the_msm_otg = dev;
	dev->vbus_on_irq = vbus_on_irq;
	dev->otg.set_peripheral = msm_otg_set_peripheral;
#ifdef CONFIG_USB_EHCI_MSM
	dev->otg.set_host = msm_otg_set_host;
#endif
	dev->otg.set_suspend = msm_otg_set_suspend;
	dev->otg.start_hnp = msm_otg_start_hnp;
	dev->otg.send_event = msm_otg_send_event;
	dev->set_clk = msm_otg_set_clk;
	dev->reset = otg_reset;
	if (otg_set_transceiver(&dev->otg)) {
		WARN_ON(1);
		goto free_otg_irq;
	}

	device_init_wakeup(&pdev->dev, 1);

	if (vbus_on_irq) {
		ret = request_irq(vbus_on_irq, pmic_vbus_on_irq,
				IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret) {
			pr_info("%s: request_irq for vbus_on"
					"interrupt failed\n", __func__);
			goto free_otg_irq;
		}
	}

#ifdef CONFIG_DEBUG_FS
	ret = otg_debugfs_init(dev);
	if (ret) {
		pr_info("%s: otg_debugfs_init failed\n", __func__);
		goto free_vbus_irq;
	}
#endif

	ret = sysfs_create_group(&pdev->dev.kobj, &msm_otg_attr_grp);
	if (ret < 0) {
		pr_err("%s: Failed to create the sysfs entry \n", __func__);
#ifdef CONFIG_DEBUG_FS
		otg_debugfs_cleanup();
#endif
		goto free_vbus_irq;
	}

	return 0;

free_vbus_irq:
	if (vbus_on_irq)
		free_irq(vbus_on_irq, 0);
free_otg_irq:
	free_irq(dev->irq, dev);
free_wq:
	destroy_workqueue(dev->wq);
free_wlock:
	wake_lock_destroy(&dev->wlock);
free_regs:
	iounmap(dev->regs);
put_phy_clk:
	if (dev->phy_reset_clk)
		clk_put(dev->phy_reset_clk);
put_hs_cclk:
	if (dev->hs_cclk)
		clk_put(dev->hs_cclk);
put_hs_pclk:
	if (dev->hs_pclk)
		clk_put(dev->hs_pclk);
put_hs_clk:
	if (dev->hs_clk)
		clk_put(dev->hs_clk);
rpc_fail:
	dev->pdata->rpc_connect(0);
free_dev:
	kfree(dev);
	return ret;
}

static int __exit msm_otg_remove(struct platform_device *pdev)
{
	struct msm_otg *dev = the_msm_otg;

#ifdef CONFIG_DEBUG_FS
	otg_debugfs_cleanup();
#endif
	sysfs_remove_group(&pdev->dev.kobj, &msm_otg_attr_grp);
	destroy_workqueue(dev->wq);
	wake_lock_destroy(&dev->wlock);

	if (dev->pmic_notif_supp)
		dev->pdata->pmic_notif_deinit();

	free_irq(dev->irq, pdev);
	if (dev->vbus_on_irq)
		free_irq(dev->irq, 0);
	iounmap(dev->regs);
	if (dev->hs_cclk) {
		clk_disable(dev->hs_cclk);
		clk_put(dev->hs_cclk);
	}
	if (dev->hs_pclk) {
		clk_disable(dev->hs_pclk);
		clk_put(dev->hs_pclk);
	}
	if (dev->hs_clk)
		clk_put(dev->hs_clk);
	if (dev->phy_reset_clk)
		clk_put(dev->phy_reset_clk);
	if (dev->pdata->rpc_connect)
		dev->pdata->rpc_connect(0);

	kfree(dev);
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

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM usb transceiver driver");
MODULE_VERSION("1.00");
