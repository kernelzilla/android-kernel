/* ehci-msm.c - HSUSB Host Controller Driver Implementation
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Partly derived from ehci-fsl.c and ehci-hcd.c
 * Copyright (c) 2000-2004 by David Brownell
 * Copyright (c) 2005 MontaVista Software
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/spinlock.h>

#include <mach/board.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_otg.h>

#define MSM_USB_BASE (hcd->regs)
#define SOC_ROC_2_0            0x10002 /* ROC 2.0 */

struct msm_hc_device {
	struct clk *clk;
	struct clk *pclk;
	struct usb_hcd *msm_hcd_data;
	unsigned phy_info;
	unsigned in_lpm;
	struct work_struct lpm_exit_work;
	struct work_struct xceiv_work;
	struct completion suspend_done;
	spinlock_t lock;
	unsigned int soc_version;
	unsigned active;
	struct msm_otg_transceiver *xceiv;
};

static struct msm_hc_device msm_hc_dev;

static void msm_vbus_online(void)
{
	pr_info("msm_hsusb: vbus online, clock enable\n");
	clk_enable(msm_hc_dev.clk);
	clk_enable(msm_hc_dev.pclk);
}

static void msm_vbus_offline(void)
{
	pr_info("msm_hsusb: vbus offline, clock disable\n");
	clk_disable(msm_hc_dev.clk);
	clk_disable(msm_hc_dev.pclk);
}

static unsigned ulpi_read(struct usb_hcd *hcd, unsigned reg)
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
		return -ETIMEDOUT;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct usb_hcd *hcd, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_write: timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int usb_wakeup_phy(struct usb_hcd *hcd)
{
	int i;

	pr_info("%s: start\n", __func__);

	writel(readl(USB_USBCMD) & ~ULPI_STP_CTRL, USB_USBCMD);

	/* some circuits automatically clear PHCD bit */
	for (i = 0; i < 5 && (readl(USB_PORTSC) & PORTSC_PHCD); i++) {
		writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC);
		msleep(1);
	}

	if ((readl(USB_PORTSC) & PORTSC_PHCD)) {
		pr_err("%s: cannot clear phcd bit\n", __func__);
		return -EAGAIN;
	}

	switch (PHY_TYPE(msm_hc_dev.phy_info)) {
	case USB_PHY_INTEGRATED:
		break;
	case USB_PHY_EXTERNAL:
		ulpi_write(hcd, 0x0A, 0x0E);
		ulpi_write(hcd, 0x0A, 0x11);
		break;
	default:
		pr_err("%s: undefined phy type\n", __func__);
		return -1;
	}

	pr_info("%s: end\n", __func__);

	return 0;
}
static int usb_suspend_phy(struct usb_hcd *hcd)
{
	int i;

	switch (PHY_TYPE(msm_hc_dev.phy_info)) {
	case USB_PHY_EXTERNAL:
		pr_info("%s: external phy\n", __func__);
		/* clear VBusValid and SessionEnd rising interrupts */
		ulpi_write(hcd, ULPI_VBUS_VALID_RAISE |
				ULPI_SESSION_END_RAISE, 0x0f);
		/* clear VBusValid and SessionEnd falling interrupts */
		ulpi_write(hcd, ULPI_VBUS_VALID_FALL |
				ULPI_SESSION_END_FALL, 0x12);

		/* spec talks about following bits in LPM but they are ignored
		 * because
		 * 1. disabling interface protection circuit: by disabling
		 * interface protection curcuit we cannot come out
		 * of lpm as asyn interrupts would be disabled
		 * 2. setting the suspendM bit: this bit would be set by usb
		 * controller once we set phcd bit.
		 */

		break;

	case USB_PHY_INTEGRATED:
		pr_info("%s: integrated phy\n", __func__);
		/* clearing latch register, keeping phy comparators ON and
		   turning off PLL are done because of h/w bugs */
		ulpi_read(hcd, 0x14);/* clear PHY interrupt latch register */
		ulpi_write(hcd, 0x01, 0x30);/* PHY comparators on in LPM */
		ulpi_write(hcd, 0x08, 0x09);/* turn off PLL on integrated phy */
		break;

	case USB_PHY_UNDEFINED:
		pr_err("%s: undefined phy type\n", __func__);
		return -1;
	}

	/* loop for 500 times with a delay of 1ms, input by AMSS USB team */
	for (i = 0; i < 500; i++) {
		/* set phy to be in lpm */
		writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);

		msleep(1);
		if (readl(USB_PORTSC) & PORTSC_PHCD)
			goto blk_stp_sig;
	}

	if (!(readl(USB_PORTSC) & PORTSC_PHCD)) {
		pr_err("unable to set phcd of portsc reg\n");
		return -1;
	}

	/* we have to set this bit again to work-around h/w bug */
	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);

blk_stp_sig:
	/* block the stop signal */
	writel(readl(USB_USBCMD) | ULPI_STP_CTRL, USB_USBCMD);

	return 0;
}

static int usb_lpm_enter(struct usb_hcd *hcd)
{
	unsigned long flags;
	struct device *dev = container_of((void *)hcd, struct device,
							driver_data);

	spin_lock_irqsave(&msm_hc_dev.lock, flags);
	if (msm_hc_dev.in_lpm == 1) {
		pr_info("%s: already in lpm. nothing to do\n", __func__);
		spin_unlock_irqrestore(&msm_hc_dev.lock, flags);
		return 0;
	}

	if (HC_IS_RUNNING(hcd->state)) {
		pr_info("%s: can't enter into lpm. controller is runnning\n",
			__func__);
		spin_unlock_irqrestore(&msm_hc_dev.lock, flags);
		return -1;
	}

	pr_info("%s: lpm enter procedure started\n", __func__);

	msm_hc_dev.in_lpm = 1;
	disable_irq(hcd->irq);
	spin_unlock_irqrestore(&msm_hc_dev.lock, flags);

	if (usb_suspend_phy(hcd)) {
		msm_hc_dev.in_lpm = 0;
		enable_irq(hcd->irq);
		pr_info("phy suspend failed\n");
		pr_info("%s: lpm enter procedure end\n", __func__);
		return -1;
	}

	/* enable async interrupt */
	writel(readl(USB_USBCMD) | ASYNC_INTR_CTRL, USB_USBCMD);

	msm_vbus_offline();
	if (msm_hc_dev.xceiv)
		msm_hc_dev.xceiv->set_suspend(1);

	if (device_may_wakeup(dev))
		enable_irq_wake(hcd->irq);
	enable_irq(hcd->irq);
	pr_info("%s: lpm enter procedure end\n", __func__);
	return 0;
}

void usb_lpm_exit_w(struct work_struct *work)
{
	struct usb_hcd *hcd = msm_hc_dev.msm_hcd_data;
	struct device *dev = container_of((void *)hcd, struct device,
							driver_data);

	msm_vbus_online();
	writel(readl(USB_USBCMD) & ~ASYNC_INTR_CTRL, USB_USBCMD);

	if (usb_wakeup_phy(hcd)) {
		pr_err("fatal error: cannot bring phy out of lpm\n");
		return;
	}

	/* If resume signalling finishes before lpm exit, PCD is not set in
	 * USBSTS register. Drive resume signal to the downstream device now
	 * so that EHCI can process the upcoming port change interrupt.*/

	writel(readl(USB_PORTSC) | PORTSC_FPR, USB_PORTSC);

	if (msm_hc_dev.xceiv)
		msm_hc_dev.xceiv->set_suspend(0);
	if (device_may_wakeup(dev))
		disable_irq_wake(hcd->irq);
	enable_irq(hcd->irq);
}


static void usb_lpm_exit(struct usb_hcd *hcd)
{
	unsigned long flags;
	spin_lock_irqsave(&msm_hc_dev.lock, flags);
	if (!msm_hc_dev.in_lpm) {
		spin_unlock_irqrestore(&msm_hc_dev.lock, flags);
		return;
	}
	msm_hc_dev.in_lpm = 0;
	disable_irq(hcd->irq);
	spin_unlock_irqrestore(&msm_hc_dev.lock, flags);
	schedule_work(&(msm_hc_dev.lpm_exit_work));
}

static irqreturn_t ehci_msm_irq(struct usb_hcd *hcd)
{
	if (!msm_hc_dev.active)
		return IRQ_HANDLED;

	if (msm_hc_dev.in_lpm == 1) {
		usb_lpm_exit(hcd);
		return IRQ_HANDLED;
	}

	if (readl(USB_OTGSC) & OTGSC_IDIS) {
		writel((OTGSC_IDIS | readl(USB_OTGSC)), USB_OTGSC);
		if (!(readl(USB_USBSTS) & CMD_RUN))
			usb_hcd_resume_root_hub(hcd);
		return IRQ_HANDLED;
	}


	return ehci_irq(hcd);
}

#ifdef CONFIG_PM

static int ehci_msm_bus_suspend(struct usb_hcd *hcd)
{
	int rc;

	rc = ehci_bus_suspend(hcd);
	if (!msm_hc_dev.active) {
		complete(&msm_hc_dev.suspend_done);
		return rc;
	}
	rc = usb_lpm_enter(hcd);
	if (rc)
		return ehci_bus_resume(hcd);
	return rc;
}

static int ehci_msm_bus_resume(struct usb_hcd *hcd)
{
	if (!msm_hc_dev.active)
		return ehci_bus_resume(hcd);
	usb_lpm_exit(hcd);
	if (cancel_work_sync(&(msm_hc_dev.lpm_exit_work)))
		usb_lpm_exit_w(NULL);
	return ehci_bus_resume(hcd);

}

#else

#define ehci_msm_bus_suspend NULL
#define ehci_msm_bus_resume NULL

#endif	/* CONFIG_PM */

static int ehci_msm_init(struct ehci_hcd *ehci)
{
	int	port	= HCS_N_PORTS(ehci->hcs_params);
	u32	__iomem	*reg_ptr;
	u32	command	= ehci_readl(ehci, &ehci->regs->command);

	/* port configuration - phy, port speed, port power, port enable */
	while (port--) {
		ehci_writel(ehci, (PORTSC_PTS_ULPI | PORT_POWER | PORT_PE),
				&ehci->regs->port_status[port]);
		msleep(20);
	}

	reg_ptr = (u32 __iomem *)(((u8 __iomem *)ehci->regs) + USBMODE);
	/* set hostmode */
	ehci_writel(ehci, (USBMODE_VBUS | USBMODE_CM_HC |
				USBMODE_SDIS), reg_ptr);

	return 0;
}

static int ehci_msm_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	ehci->caps = USB_CAPLENGTH;
	ehci->regs = USB_CAPLENGTH +
		HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

	/* cache the data to minimize the chip reads*/
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	retval = ehci_init(hcd);
	if (retval)
		return retval;

	hcd->has_tt = 1;

	ehci->sbrn = HCD_USB2;

	ehci_reset(ehci);

	retval = ehci_msm_init(ehci);

	return retval;
}

/* SW workarounds
 * Issue#2		- Integrated PHY Calibration
 * Symptom		- Electrical compliance failure in eye-diagram tests
 * SW workaround        - Try to raise amplitude to 400mV

 * Issue#3		- AHB Posted Writes
 * Symptom		- USB stability
 * SW workaround	- This programs xtor ON, BURST disabled and
 *			unspecified length of INCR burst enabled
 */
static int ehci_msm_run(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci  = hcd_to_ehci(hcd);
	int             retval = 0;
	int     	port   = HCS_N_PORTS(ehci->hcs_params);
	u32 __iomem     *reg_ptr;
	u32             hcc_params;

	hcd->uses_new_polling = 1;
	hcd->poll_rh = 0;

	/* EHCI spec section 4.1 */
	retval = ehci_reset(ehci);
	if (retval) {
		ehci_mem_cleanup(ehci);
		return retval;
	}

	/* set hostmode */
	reg_ptr = (u32 __iomem *)(((u8 __iomem *)ehci->regs) + USBMODE);
	ehci_writel(ehci, (USBMODE_VBUS | USBMODE_CM_HC |
				USBMODE_SDIS), reg_ptr);

	/* port configuration - phy, port speed, port power, port enable */
	while (port--) {
		ehci_writel(ehci, (PORTSC_PTS_ULPI | PORT_POWER | PORT_PE),
				&ehci->regs->port_status[port]);
		msleep(20);
	}

	if (PHY_TYPE(msm_hc_dev.phy_info) == USB_PHY_INTEGRATED) {
		/* SW workaround, Issue#2 */
		retval = ulpi_write(hcd, ULPI_AMPLITUDE, ULPI_CONFIG_REG);
		if (retval)
			return retval;
		/* SW workaround, Issue#3 */
		writel(0x0, USB_AHB_MODE);
		writel(0x0, USB_AHB_BURST);
	} else {
		if (msm_hc_dev.soc_version >= SOC_ROC_2_0)
			writel(0x02, USB_ROC_AHB_MODE);
		else
			writel(0x01, USB_ROC_AHB_MODE);
	}

	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);

	hcc_params = ehci_readl(ehci, &ehci->caps->hcc_params);
	if (HCC_64BIT_ADDR(hcc_params))
		ehci_writel(ehci, 0, &ehci->regs->segment);

	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/*
	 * Start, enabling full USB 2.0 functionality ... usb 1.1 devices
	 * are explicitly handed to companion controller(s), so no TT is
	 * involved with the root hub.  (Except where one is integrated,
	 * and there's no companion controller unless maybe for USB OTG.)
	 *
	 * Turning on the CF flag will transfer ownership of all ports
	 * from the companions to the EHCI controller.  If any of the
	 * companions are in the middle of a port reset at the time, it
	 * could cause trouble.  Write-locking ehci_cf_port_reset_rwsem
	 * guarantees that no resets are in progress.  After we set CF,
	 * a short delay lets the hardware catch up; new resets shouldn't
	 * be started before the port switching actions could complete.
	 */

	down_write(&ehci_cf_port_reset_rwsem);
	hcd->state = HC_STATE_RUNNING;
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command); /* unblock posted writes */
	msleep(5);
	up_write(&ehci_cf_port_reset_rwsem);

	/*Enable appropriate Interrupts*/
	ehci_writel(ehci, INTR_MASK,
			&ehci->regs->intr_enable);

	writel(readl(USB_OTGSC) | OTGSC_IDIE, USB_OTGSC);
	return retval;
}

static struct hc_driver msm_hc_driver = {
	.description		= hcd_name,
	.product_desc 		= "Qualcomm On-Chip EHCI Host Controller",
	.hcd_priv_size 		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq 			= ehci_msm_irq,
	.flags 			= HCD_USB2,

	.reset 			= ehci_msm_reset,
	.start 			= ehci_msm_run,

	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_msm_bus_suspend,
	.bus_resume		= ehci_msm_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
};

static void ehci_msm_enable(int enable)
{
	int prev_state;
	struct usb_hcd *hcd = msm_hc_dev.msm_hcd_data;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	if (enable) {
		msm_vbus_online();
		msm_hsusb_vbus_powerup();
		prev_state = hcd->state;
		ehci_reset(ehci);
		ehci_msm_run(hcd);
		hcd->state = prev_state;
		usb_hcd_resume_root_hub(hcd);
		msm_hc_dev.active = 1;
	} else {
		msm_hc_dev.active = 0;
		msm_hsusb_vbus_shutdown();
		if (hcd->state != HC_STATE_SUSPENDED)
			wait_for_completion(&msm_hc_dev.suspend_done);
		msm_vbus_offline();
	}
}

static struct msm_otg_ops hcd_ops = {
	.status_change = ehci_msm_enable,
};

void usb_xceiv_work(struct work_struct *work)
{
	int retval;

	retval = msm_hc_dev.xceiv->set_host(msm_hc_dev.xceiv,
			&hcd_ops);
	if (retval)
		pr_err("%s: Can't register Host driver with OTG",
				__func__);
}
static int __init ehci_msm_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	struct hc_driver *driver = &msm_hc_driver;
	int irq;
	int retval;
	int msec;
	u32 result;

	msm_hc_dev.msm_hcd_data = NULL;
       /* initializing phy type */
	if (pdev->dev.platform_data) {
		struct msm_hsusb_platform_data *pdata = pdev->dev.platform_data;
		msm_hc_dev.soc_version = pdata->soc_version;
		msm_hc_dev.phy_info = pdata->phy_info;
		if (PHY_TYPE(msm_hc_dev.phy_info) == USB_PHY_UNDEFINED) {
			printk(KERN_ERR "undefined phytype:(%d)\n",
			      msm_hc_dev.phy_info);
			return -ENODEV;
		}
	} else
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ALERT "Found HC with no IRQ, \
					check %s set up!\n", pdev->dev.bus_id);
		retval = -ENODEV;
		goto err_hcd_create;
	}

	irq = res->start;

	hcd = usb_create_hcd(driver, &pdev->dev, pdev->dev.bus_id);
	if (!hcd) {
		printk(KERN_ALERT "failed @ usb_create_hcd\n");
		retval = -ENOMEM;
		goto err_hcd_create;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ALERT "Found HC with no register address, \
					check %s setup!\n", pdev->dev.bus_id);
		retval = -ENODEV;
		goto err_no_regs;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
						driver->description)) {
		printk(KERN_ALERT "Controller already in use!!!\n");
		retval = -EBUSY;
		goto err_res_busy;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (hcd->regs == NULL) {
		printk(KERN_ALERT "Error mapping the memory\n");
		retval = -EFAULT;
		goto err_map;
	}

	/* get usb clocks */
	msm_hc_dev.clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(msm_hc_dev.clk)) {
		printk(KERN_ALERT "usb_hs_clk error\n");
		retval = -EFAULT;
		goto err_map;
	}

	msm_hc_dev.pclk = clk_get(&pdev->dev, "usb_hs_pclk");
	if (IS_ERR(msm_hc_dev.pclk)) {
		printk(KERN_ALERT "usb_hs_pclk error\n");
		retval = -EFAULT;
		goto err_map;
	}

	/* enable usb clocks */
	msm_vbus_online();

	msm_hsusb_rpc_connect();

	retval = msm_hsusb_phy_reset();

	if (retval < 0)
		goto err_hcd_add;

	/* enable IDpullup to start ID pin sampling */
	ulpi_write(hcd, ULPI_IDPU, ULPI_OTG_CTRL);
	/* Disable VbusValid and SessionEnd comparators */
	ulpi_write(hcd, ULPI_VBUS_VALID | ULPI_SESS_END, ULPI_INT_RISE_CLR);
	ulpi_write(hcd, ULPI_VBUS_VALID | ULPI_SESS_END, ULPI_INT_FALL_CLR);


	/* reset hc core */
	writel((readl(USB_USBCMD) | CMD_RESET), USB_USBCMD);

	/* waiting for hc core reset upto 250ms */
	for (msec = 250; msec > 0; msec--) {
		result = readl(USB_USBCMD);
		if (result == ~(u32)0) {
			retval = -ENODEV;
			goto err_hcd_add;
		}

		if ((result & CMD_RESET) == 0)
			break;

		msleep(1);
	}
	if (msec <= 0) {
		retval = -ETIMEDOUT;
		goto err_hcd_add;
	}

	/* set hostmode */
	writel((USBMODE_VBUS | USBMODE_CM_HC), USB_USBMODE);
	msm_hc_dev.in_lpm = 0;
	spin_lock_init(&msm_hc_dev.lock);
	INIT_WORK(&(msm_hc_dev.lpm_exit_work), usb_lpm_exit_w);
	INIT_WORK(&(msm_hc_dev.xceiv_work), usb_xceiv_work);
	init_completion(&msm_hc_dev.suspend_done);
	device_init_wakeup(&pdev->dev, 1);
	msm_hc_dev.active = 1;

	/* disable interrupts before requesting irq */
	msm_vbus_online();
	writel(0, USB_USBINTR);
	writel(readl(USB_OTGSC) & ~OTGSC_INTR_MASK, USB_OTGSC);
	msm_vbus_offline();

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval != 0)
		goto err_hcd_add;

	msm_hc_dev.msm_hcd_data = hcd;

	msm_hc_dev.xceiv = msm_otg_get_transceiver();
	if (msm_hc_dev.xceiv) {
		msm_hc_dev.active = 0;
		schedule_work(&(msm_hc_dev.xceiv_work));
	} else {
		msm_hsusb_vbus_powerup();
	}

	return retval;

err_hcd_add:
	iounmap(hcd->regs);
err_map:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err_res_busy:
err_no_regs:
	usb_put_hcd(hcd);
err_hcd_create:
	msm_hsusb_rpc_close();
	msm_vbus_offline();
	printk(KERN_ALERT "Init %s fail, %d\n", pdev->dev.bus_id, retval);
	return retval;
}

static int __exit ehci_msm_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = msm_hc_dev.msm_hcd_data;
	int retval = 0;

	cancel_work_sync(&msm_hc_dev.lpm_exit_work);
	device_init_wakeup(&pdev->dev, 0);
	if (msm_hc_dev.msm_hcd_data) {
		usb_remove_hcd(hcd);
		iounmap(hcd->regs);
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
		usb_put_hcd(hcd);
	}

	if (msm_hc_dev.xceiv) {
		msm_hc_dev.xceiv->set_host(msm_hc_dev.xceiv, NULL);
		msm_otg_put_transceiver(msm_hc_dev.xceiv);
	}

	retval = msm_hsusb_vbus_shutdown();
	if (retval < 0)
		return retval;

	retval = msm_hsusb_rpc_close();
	if (retval < 0)
		return retval;

	msm_vbus_offline();
	clk_put(msm_hc_dev.clk);
	clk_put(msm_hc_dev.pclk);

	return retval;
}

static struct platform_driver ehci_msm_driver = {
	.probe	= ehci_msm_probe,
	.remove	= __exit_p(ehci_msm_remove),
	.driver	= {.name = "msm_hsusb_host"},
};
