/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/usb/omap.h>

#include <mach/gpio.h>

#include "ehci-omap.h"

/* These default platform_data configs are an attempt to maintain
 * backwards compatiblity with the old driver.  These contains
 * board specific data and should be migrated the the apropriate
 * board files.
 */

#ifdef CONFIG_OMAP_EHCI_PHY_MODE
/* EHCI connected to External PHY */

/* External USB connectivity board: 750-2083-001
 * Connected to OMAP3430 SDP
 * The board has Port1 and Port2 connected to ISP1504 in 12-pin ULPI mode
 */

/* ISSUE1:
 *      ISP1504 for input clocking mode needs special reset handling
 *	Hold the PHY in reset by asserting RESET_N signal
 *	Then start the 60Mhz clock input to PHY
 *	Release the reset after a delay -
 *		to get the PHY state machine in working state
 */

#define	EXT_PHY_RESET_GPIO_PORT1	(57)
#define	EXT_PHY_RESET_GPIO_PORT2	(61)

static int default_usb_port_startup(struct platform_device *dev, int port)
{
	int r;
	int gpio;
	const char *name;

	if (port == 0) {
		gpio = EXT_PHY_RESET_GPIO_PORT1;
		name = "ehci port 1 reset";
	} else if (port == 1) {
		gpio = EXT_PHY_RESET_GPIO_PORT2;
		name = "ehci port 2 reset";
	} else {
		return -EINVAL;
	}

	r = gpio_request(gpio, name);
	if (r < 0) {
		printk(KERN_WARNING "Could not request GPIO %d"
		       " for port %d reset\n",
		       gpio, port);
		return r;
	}
	gpio_direction_output(gpio, 0);
	return 0;
}

static void default_usb_port_shutdown(struct platform_device *dev, int port)
{
	if (port == 0)
		gpio_free(EXT_PHY_RESET_GPIO_PORT1);
	else if (port == 1)
		gpio_free(EXT_PHY_RESET_GPIO_PORT2);
}


static void default_usb_port_reset(struct platform_device *dev,
				   int port, int reset)
{
	if (port == 0)
		gpio_set_value(EXT_PHY_RESET_GPIO_PORT1, !reset);
	else if (port == 1)
		gpio_set_value(EXT_PHY_RESET_GPIO_PORT2, !reset);
}


static struct omap_usb_port_data default_usb_port_data[] = {
	[0] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED,
		.mode = OMAP_USB_PORT_MODE_ULPI_PHY,
		.reset_delay = 10,
		.startup = default_usb_port_startup,
		.shutdown = default_usb_port_shutdown,
		.reset = default_usb_port_reset,
	},
	[1] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED,
		.mode = OMAP_USB_PORT_MODE_ULPI_PHY,
		.reset_delay = 10,
		.startup = default_usb_port_startup,
		.shutdown = default_usb_port_shutdown,
		.reset = default_usb_port_reset,
	},
	[2] = { .flags = 0x0, }, /* disabled */

};

/* ISSUE2:
 * USBHOST supports External charge pump PHYs only
 * Use the VBUS from Port1 to power VBUS of Port2 externally
 * So use Port2 as the working ULPI port
 */
static struct omap_usb_platform_data default_usb_platform_data = {
	.flags =  OMAP_USB_FLAG_VBUS_INTERNAL_CHARGEPUMP,
	.port_data = default_usb_port_data,
	.num_ports = ARRAY_SIZE(default_usb_port_data),
};

#else /* CONFIG_OMAP_EHCI_PHY_MODE */

static struct omap_usb_port_data default_usb_port_data[] = {
	[0] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED,
		.mode = OMAP_USB_PORT_MODE_UTMI_PHY_6PIN,
	},
	[1] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED,
		.mode = OMAP_USB_PORT_MODE_ULPI_PHY,
	},
	[2] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED,
		.mode = OMAP_USB_PORT_MODE_ULPI_PHY,
	},
};

static struct omap_usb_platform_data default_usb_platform_data = {
	.port_data = default_usb_port_data,
	.num_ports = ARRAY_SIZE(default_usb_port_data),
};

#endif /* CONFIG_OMAP_EHCI_PHY_MODE */


/*-------------------------------------------------------------------------*/

/* Define USBHOST clocks for clock management */
struct ehci_omap_clock_defs {
	struct clk	*usbhost_ick_clk;
	struct clk	*usbhost2_120m_fck_clk;
	struct clk	*usbhost1_48m_fck_clk;
	struct clk	*usbtll_fck_clk;
	struct clk	*usbtll_ick_clk;
};

/* Clock names as per clock framework: May change so keep as #defs */
#define USBHOST_ICKL		"usbhost_ick"
#define USBHOST_120M_FCLK	"usbhost_120m_fck"
#define USBHOST_48M_FCLK	"usbhost_48m_fck"
#define USBHOST_TLL_ICKL	"usbtll_ick"
#define USBHOST_TLL_FCLK	"usbtll_fck"
/*-------------------------------------------------------------------------*/

static int omap_usb_port_ulpi_bypass(enum omap_usb_port_mode mode)
{
	return mode != OMAP_USB_PORT_MODE_ULPI_PHY;
}

static int omap_usb_port_ttl_chanel_config(enum omap_usb_port_mode mode)
{
	return (mode == OMAP_USB_PORT_MODE_UTMI_PHY_6PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_PHY_6PIN_ALT) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_PHY_3PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_PHY_4PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_6PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_6PIN_ALT) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_3PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_4PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_2PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_2PIN_ALT);
}

static unsigned omap_usb_port_fslsmode(enum omap_usb_port_mode mode)
{
	switch (mode) {
		/* non serial modes */
	case OMAP_USB_PORT_MODE_ULPI_PHY:
	case OMAP_USB_PORT_MODE_ULPI_TLL_SDR:
	case OMAP_USB_PORT_MODE_ULPI_TLL_DDR:
		return 0x0;

		/* serial modes */
	case OMAP_USB_PORT_MODE_UTMI_PHY_6PIN:
		return 0x0;

	case OMAP_USB_PORT_MODE_UTMI_PHY_6PIN_ALT:
		return 0x1;

	case OMAP_USB_PORT_MODE_UTMI_PHY_3PIN:
		return 0x2;

	case OMAP_USB_PORT_MODE_UTMI_PHY_4PIN:
		return 0x3;

	case OMAP_USB_PORT_MODE_UTMI_TLL_6PIN:
		return 0x4;

	case OMAP_USB_PORT_MODE_UTMI_TLL_6PIN_ALT:
		return 0x5;

	case OMAP_USB_PORT_MODE_UTMI_TLL_3PIN:
		return 0x6;

	case OMAP_USB_PORT_MODE_UTMI_TLL_4PIN:
		return 0x7;

	case OMAP_USB_PORT_MODE_UTMI_TLL_2PIN:
		return 0xa;

	case OMAP_USB_PORT_MODE_UTMI_TLL_2PIN_ALT:
		return 0xB;
	}

	return 0x0;
}

static void omap_usb_setup_ports(struct usb_hcd *hcd)
{
	int i;
	struct omap_usb_platform_data *data =  (struct omap_usb_platform_data *)
		hcd->self.controller->platform_data;
	u32 val;

	val = (1<<OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT)|
		(1<<OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT)|
		(1<<OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT)|
		(0<<OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT);


	if ((data->num_ports > 0) &&
	    (omap_usb_port_ulpi_bypass(data->port_data[0].mode)))
		val |= (1 << OMAP_UHH_HOSTCONFIG_P1_ULPI_BYPASS_SHIFT);
	if ((data->num_ports > 1) &&
	    (omap_usb_port_ulpi_bypass(data->port_data[1].mode)))
		val |= (1 << OMAP_UHH_HOSTCONFIG_P2_ULPI_BYPASS_SHIFT);
	if ((data->num_ports > 2) &&
	    (omap_usb_port_ulpi_bypass(data->port_data[2].mode)))
		val |= (1 << OMAP_UHH_HOSTCONFIG_P3_ULPI_BYPASS_SHIFT);

	omap_writel(val, OMAP_UHH_HOSTCONFIG);

	/* wait for change to sync */
	while ((omap_readl(OMAP_UHH_HOSTCONFIG) &
		OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_MASK) !=
	       (val & OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_MASK))
		cpu_relax();

	dev_dbg(hcd->self.controller, "ULPI Byppass mode congfigured: %08x\n",
		omap_readl(OMAP_UHH_HOSTCONFIG));

	/* Program the 3 channels upfront */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		/* AutoIdle */
		if ((data->num_ports > i) &&
		    (data->port_data[i].flags & OMAP_USB_PORT_FLAG_AUTOIDLE))
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
				    1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT,
				    OMAP_TLL_CHANNEL_CONF(i));
		else
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
				    ~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));

		/* BitStuffing */
		if ((data->num_ports > i) &&
		    (data->port_data[i].flags & OMAP_USB_PORT_FLAG_NOBITSTUFF))
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
				    1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT,
				    OMAP_TLL_CHANNEL_CONF(i));
		else
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
				    ~(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));

		/* SDR/DDR */
		if ((data->num_ports > i) &&
		    (data->port_data[i].mode == OMAP_USB_PORT_MODE_ULPI_TLL_DDR))
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
				    (1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));
		else
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
				    ~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));
	}

	/* Program Common TLL register */
	omap_writel((1 << OMAP_TLL_SHARED_CONF_FCLK_IS_ON_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_DIVRATION_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN_SHFT),
				OMAP_TLL_SHARED_CONF);

	/* Enable channels */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		u32 channel_conf;

		if ((data->num_ports <= i) ||
		   !(data->port_data[i].flags & OMAP_USB_PORT_FLAG_ENABLED)) {
			dev_dbg(hcd->self.controller,
				"port %d disabled. skipping.\n", i);
			continue;
		}

		channel_conf = omap_readl(OMAP_TLL_CHANNEL_CONF(i));
		channel_conf |= 1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT;

		if (omap_usb_port_ttl_chanel_config(data->port_data[i].mode))
			channel_conf |= 1<<OMAP_TLL_CHANNEL_CONF_CHANMODE_SHIFT;
		else
			channel_conf &= ~(3<<OMAP_TLL_CHANNEL_CONF_CHANMODE_SHIFT);

		channel_conf &= ~(OMAP_TLL_CHANNEL_CONF_FSLSMODE(0xf));
		channel_conf |= OMAP_TLL_CHANNEL_CONF_FSLSMODE(
			omap_usb_port_fslsmode(data->port_data[i].mode));


		omap_writel(channel_conf, OMAP_TLL_CHANNEL_CONF(i));
		dev_dbg(hcd->self.controller,
			"port %d enabled: OMAP_TTL_CHANNEL_CONF_%d=%08x\n",
			i, i+1, omap_readl(OMAP_TLL_CHANNEL_CONF(i)));

		omap_writeb(0xBE, OMAP_TLL_ULPI_SCRATCH_REGISTER(i));
		dev_dbg(hcd->self.controller, "ULPI_SCRATCH_REG[ch=%d]"
			"= 0x%02x\n",
			i+1, omap_readb(OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}

/* omap_start_ehc
 *	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;
	struct omap_usb_platform_data *data = dev->dev.platform_data;
	int i, r, reset_delay;

	dev_dbg(hcd->self.controller, "starting TI EHCI USB Controller\n");

	ehci_clocks = (struct ehci_omap_clock_defs *)(
				((char *)hcd_to_ehci(hcd)) +
					sizeof(struct ehci_hcd));

	/* Start DPLL5 Programming:
	 * Clock Framework is not doing this now:
	 * This will be done in clock framework later
	 */
	/* Enable DPLL 5 : Based on Input of 13Mhz*/
	cm_write_mod_reg((12 << OMAP3430ES2_PERIPH2_DPLL_DIV_SHIFT)|
			(120 << OMAP3430ES2_PERIPH2_DPLL_MULT_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKSEL4);

	cm_write_mod_reg(1 << OMAP3430ES2_DIV_120M_SHIFT,
			PLL_MOD, OMAP3430ES2_CM_CLKSEL5);

	cm_write_mod_reg((7 << OMAP3430ES2_PERIPH2_DPLL_FREQSEL_SHIFT) |
			(7 << OMAP3430ES2_EN_PERIPH2_DPLL_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKEN2);

	while (!(cm_read_mod_reg(PLL_MOD, CM_IDLEST2) &
				OMAP3430ES2_ST_PERIPH2_CLK_MASK))
		dev_dbg(hcd->self.controller,
			"idlest2 = 0x%x\n",
			cm_read_mod_reg(PLL_MOD, CM_IDLEST2));
	/* End DPLL5 programming */


	/* PRCM settings for USBHOST:
	 * Interface clk un-related to domain transition
	 */
	cm_write_mod_reg(0 << OMAP3430ES2_AUTO_USBHOST_SHIFT,
				OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE);

	/* Disable sleep dependency with MPU and IVA */
	cm_write_mod_reg((0 << OMAP3430ES2_EN_MPU_SHIFT) |
				(0 << OMAP3430ES2_EN_IVA2_SHIFT),
				OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);

	/* Disable Automatic transition of clock */
	cm_write_mod_reg(0 << OMAP3430ES2_CLKTRCTRL_USBHOST_SHIFT,
				OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL);

	/* Enable Clocks for USBHOST */
	ehci_clocks->usbhost_ick_clk = clk_get(&dev->dev,
						USBHOST_ICKL);
	if (IS_ERR(ehci_clocks->usbhost_ick_clk))
		return PTR_ERR(ehci_clocks->usbhost_ick_clk);
	clk_enable(ehci_clocks->usbhost_ick_clk);


	ehci_clocks->usbhost2_120m_fck_clk = clk_get(&dev->dev,
							USBHOST_120M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost2_120m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost2_120m_fck_clk);
	clk_enable(ehci_clocks->usbhost2_120m_fck_clk);

	ehci_clocks->usbhost1_48m_fck_clk = clk_get(&dev->dev,
						USBHOST_48M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost1_48m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost1_48m_fck_clk);
	clk_enable(ehci_clocks->usbhost1_48m_fck_clk);

	reset_delay = 0;
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if ((data->num_ports > i) && (data->port_data[i].startup)) {
			reset_delay = reset_delay > data->port_data[i].reset_delay ?
				reset_delay : data->port_data[i].reset_delay;
			r = data->port_data[i].startup(dev, i);
			if (r < 0)
				return r;
		}
	}
	if (reset_delay)
		udelay(reset_delay);

	/* Configure TLL for 60Mhz clk for ULPI */
	ehci_clocks->usbtll_fck_clk = clk_get(&dev->dev, USBHOST_TLL_FCLK);
	if (IS_ERR(ehci_clocks->usbtll_fck_clk))
		return PTR_ERR(ehci_clocks->usbtll_fck_clk);
	clk_enable(ehci_clocks->usbtll_fck_clk);

	ehci_clocks->usbtll_ick_clk = clk_get(&dev->dev, USBHOST_TLL_ICKL);
	if (IS_ERR(ehci_clocks->usbtll_ick_clk))
		return PTR_ERR(ehci_clocks->usbtll_ick_clk);
	clk_enable(ehci_clocks->usbtll_ick_clk);

	/* Disable Auto Idle of USBTLL */
	cm_write_mod_reg((0 << OMAP3430ES2_AUTO_USBTLL_SHIFT),
				CORE_MOD, CM_AUTOIDLE3);

	/* Wait for TLL to be Active */
	while ((cm_read_mod_reg(CORE_MOD, OMAP2430_CM_IDLEST3)
			& (1 << OMAP3430ES2_ST_USBTLL_SHIFT)))
		cpu_relax();

	/* perform TLL soft reset, and wait until reset is complete */
	omap_writel(1 << OMAP_USBTLL_SYSCONFIG_SOFTRESET_SHIFT,
			OMAP_USBTLL_SYSCONFIG);
	/* Wait for TLL reset to complete */
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS)
			& (1 << OMAP_USBTLL_SYSSTATUS_RESETDONE_SHIFT)))
		cpu_relax();

	dev_dbg(hcd->self.controller, "TLL RESET DONE\n");

	/* (1<<3) = no idle mode only for initial debugging */
	omap_writel((1 << OMAP_USBTLL_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_SIDLEMODE_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_CACTIVITY_SHIFT),
			OMAP_USBTLL_SYSCONFIG);


	/* Put UHH in NoIdle/NoStandby mode */
	omap_writel((0 << OMAP_UHH_SYSCONFIG_AUTOIDLE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_SIDLEMODE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_CACTIVITY_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_MIDLEMODE_SHIFT),
			OMAP_UHH_SYSCONFIG);

	omap_usb_setup_ports(hcd);

	reset_delay = 0;
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if (data->num_ports > i)
			reset_delay = reset_delay > data->port_data[i].reset_delay ?
				reset_delay : data->port_data[i].reset_delay;
	}
	if (reset_delay)
		udelay(reset_delay);

	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if ((data->num_ports > i) && (data->port_data[i].reset))
			data->port_data[i].reset(dev, i, 1);
	}

	if (data->flags & OMAP_USB_FLAG_VBUS_INTERNAL_CHARGEPUMP) {
		/* Refer ISSUE2: LINK assumes external charge pump */

		/* use Port1 VBUS to charge externally Port2:
		 *	So for PHY mode operation use Port2 only
		 */
		omap_writel((0xA << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |/* OTG ctrl reg*/
			    (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |/*   Write */
			    (1 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port1 */
			    (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) |/* Start */
			    (0x26),
			    EHCI_INSNREG05_ULPI);

		while (!(omap_readl(EHCI_INSNREG05_ULPI)
			 & (1<<EHCI_INSNREG05_ULPI_CONTROL_SHIFT)))
			cpu_relax();
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static void omap_stop_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;
	struct omap_usb_platform_data *data = dev->dev.platform_data;
	int i;

	ehci_clocks = (struct ehci_omap_clock_defs *)
			(((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	dev_dbg(hcd->self.controller, "stopping TI EHCI USB Controller\n");

	/* Reset OMAP modules for insmod/rmmod to work */
	omap_writel((1<<1), OMAP_UHH_SYSCONFIG);
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<0)))
		cpu_relax();
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<1)))
		cpu_relax();
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<2)))
		cpu_relax();
	dev_dbg(hcd->self.controller,
		"UHH RESET DONE OMAP_UHH_SYSSTATUS %x !!\n",
			omap_readl(OMAP_UHH_SYSSTATUS));

	omap_writel((1<<1), OMAP_USBTLL_SYSCONFIG);
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) & (1<<0)))
		cpu_relax();
	dev_dbg(hcd->self.controller, "TLL RESET DONE\n");

	if (ehci_clocks->usbtll_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_fck_clk);
		clk_put(ehci_clocks->usbtll_fck_clk);
		ehci_clocks->usbtll_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbhost_ick_clk);
		clk_put(ehci_clocks->usbhost_ick_clk);
		ehci_clocks->usbhost_ick_clk = NULL;
	}

	if (ehci_clocks->usbhost1_48m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost1_48m_fck_clk);
		clk_put(ehci_clocks->usbhost1_48m_fck_clk);
		ehci_clocks->usbhost1_48m_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost2_120m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost2_120m_fck_clk);
		clk_put(ehci_clocks->usbhost2_120m_fck_clk);
		ehci_clocks->usbhost2_120m_fck_clk = NULL;
	}

	if (ehci_clocks->usbtll_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_ick_clk);
		clk_put(ehci_clocks->usbtll_ick_clk);
		ehci_clocks->usbtll_ick_clk = NULL;
	}

	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if ((data->num_ports > i) && (data->port_data[i].shutdown))
			data->port_data[i].shutdown(dev, i);
	}

	dev_dbg(hcd->self.controller,
		"Clock to USB host has been disabled\n");
}

static const struct hc_driver ehci_omap_hc_driver;

/*-------------------------------------------------------------------------*/
/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_drv_probe - initialize TI-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
static int ehci_hcd_omap_drv_probe(struct platform_device *dev)
{
	int retval = 0;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;

	dev_dbg(&dev->dev, "ehci_hcd_omap_drv_probe()\n");

	if (usb_disabled())
		return -ENODEV;

	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		dev_dbg(&dev->dev, "resource[1] is not IORESOURCE_IRQ\n");
		retval = -ENOMEM;
	}

	if (dev->dev.platform_data == NULL)
		dev->dev.platform_data = &default_usb_platform_data;

	hcd = usb_create_hcd(&ehci_omap_hc_driver, &dev->dev, dev->dev.bus_id);
	if (!hcd)
		return -ENOMEM;

	retval = omap_start_ehc(dev, hcd);
	if (retval)
		return retval;

	hcd->rsrc_start = 0;
	hcd->rsrc_len = 0;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&dev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	ehci->sbrn = 0x20;

	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	/* SET 1 micro-frame Interrupt interval */
	writel(readl(&ehci->regs->command) | (1<<16), &ehci->regs->command);

	retval = usb_add_hcd(hcd, dev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	dev_dbg(hcd->self.controller, "ERR: add_hcd\n");
	omap_stop_ehc(dev, hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);

	return retval;
}

/*-------------------------------------------------------------------------*/

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ehci_hcd_omap_drv_remove - shutdown processing for EHCI HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static int ehci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);

	dev_dbg(&dev->dev, "ehci_hcd_omap_drv_remove()\n");

	iounmap(hcd->regs);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	omap_stop_ehc(dev, hcd);

	return 0;
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int omap_ehci_bus_suspend(struct usb_hcd *hcd)
{
	return ehci_bus_suspend(hcd);
}

static int omap_ehci_bus_resume(struct usb_hcd *hcd)
{
	return ehci_bus_resume(hcd);
}
#endif
/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver = {
	.description = hcd_name,
	.product_desc = "OMAP-EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd)
				+ sizeof(struct ehci_omap_clock_defs),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend = omap_ehci_bus_suspend,
	.bus_resume = omap_ehci_bus_resume,
#endif
};

/*-------------------------------------------------------------------------*/
MODULE_ALIAS("platform:omap-ehci");
static struct platform_driver ehci_hcd_omap_driver = {
	.probe = ehci_hcd_omap_drv_probe,
	.remove = ehci_hcd_omap_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	/*.suspend      = ehci_hcd_omap_drv_suspend, */
	/*.resume       = ehci_hcd_omap_drv_resume, */
	.driver = {
		.name = "ehci-omap",
		.bus = &platform_bus_type
	}
};
